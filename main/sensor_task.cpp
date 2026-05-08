/**
 * sensor_task.cpp — VL53L0X ToF array driver + swipe-to-note translator.
 *
 * Adapted from Lennarts_code_base/Lens_sensors/main/main.cpp (the proven
 * 8-sensor / 74HC595 / 22 ms cadence layout). What we keep:
 *   - 74HC595 sequential XSHUT bring-up so each sensor gets a unique
 *     I2C address (0x30..0x37).
 *   - Continuous-ranging mode at a 20 ms timing budget; 22 ms poll
 *     period gives a small margin above the budget.
 *   - VL53L0X C++ wrapper from the components/vl53l0x driver.
 *
 * What we remove:
 *   - LED strip + metronome (lives elsewhere).
 *   - Audio mixer / WAV playback (the existing block-based audio
 *     pipeline owns that now).
 *
 * What we add:
 *   - Per-sensor swipe detector. We track the previous distance and
 *     timestamp; when the hand crosses TRIGGER_DIST going downward
 *     AND the smoothed approach speed exceeds MIN_SWIPE_SPEED we post
 *     one note_event onto g_note_queue.
 *   - Per-sensor pitch: each sensor index maps to a degree of the
 *     C-major scale, expressed as a playback-speed ratio.
 *   - Per-instrument retrigger lock-out so a single fast swipe doesn't
 *     re-fire on the next 22 ms tick.
 *
 * Threading: this whole file is one producer task. ToF sample swipes post
 * to g_note_queue only in MODE_SAMPLE. MODE_SYNTH updates tracks: live ToF on active_track,
 * loop playback on others; while is_recording, commits sustained NOTE_SOURCE_SYNTH_NOTE
 * segments (start time, pitch, start velocity, duration) to the looper.
 */

#include "sensor_task.h"
#include "shared_state.h"
#include "loop_recorder.h"
#include "led_task.h"
#include "panel_input.h"

#include <math.h>
#include <string.h>
#include <cstdint>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "VL53L0X.h"

static const char *TAG = "sensor_task";

/* ------------------------------------------------------------------ */
/* Pinout                                                              */
/* ------------------------------------------------------------------ */
/* Match the wiring documented in the project README:                  */
/*     74HC595 SHCP -> GPIO 12   (shift clock)                         */
/*     74HC595 STPC -> GPIO 13   (latch / storage clock)               */
/*     74HC595 DS   -> GPIO 14   (serial data)                         */
/*     CLEAR pulled to 3.3V, OE (DE) pulled to GND.                    */
/*     Outputs Q0..Q7 wired to XSHUT of ToF 0..7.                      */

#define SHIFT_SHCP_PIN      GPIO_NUM_12
#define SHIFT_STPC_PIN      GPIO_NUM_13
#define SHIFT_DS_PIN        GPIO_NUM_14

/* Adafruit Feather V2 (ESP32) labels for SDA/SCL */
#define I2C_SDA_PIN         GPIO_NUM_22
#define I2C_SCL_PIN         GPIO_NUM_20
#define I2C_FREQ_HZ         400000

/* ------------------------------------------------------------------ */
/* Sensor configuration                                                */
/* ------------------------------------------------------------------ */

#define SENSOR_POLL_MS      22       /* > 20 ms timing budget */
#define SENSOR_TIMING_US    20000    /* 20 ms high-speed budget */

/* Distance gates (mm). Tuned for a hand swiping a few cm above the
 * sensor face. */
#define DIST_ARM_MM         260      /* hand must rise above this to re-arm */
#define DIST_TRIGGER_MM     220      /* crossing below this fires a note */
#define DIST_RANGE_MAX      1000     /* anything farther = "no hand" */

/* Lateral swipe (gliss) detection:
 * Treat neighboring sensor crossings as horizontal travel and estimate
 * lateral speed from sensor index delta over time. */
#define SENSOR_PITCH_MM             114.0f   /* effective center-to-center spacing */
#define GLISS_MIN_LATERAL_MMPS      400.0f  /* minimum swipe speed for gliss */
#define GLISS_MIN_SENSORS           3       /* must cross at least this many ToFs */
#define GLISS_RUN_TIMEOUT_US        180000  /* reset run if crossings too sparse */

/* Velocity → MIDI velocity (sample volume) mapping.
 * Approach speed in mm/sec:
 *   100  mm/s  -> roughly noise floor, ignored
 *   400  mm/s  -> mezzo-piano (~80)
 *   1500 mm/s  -> fortissimo (255)
 * Anything above that clips to 255. */
#define MIN_SWIPE_SPEED_MMPS    100.0f
#define MAX_SWIPE_SPEED_MMPS    1250.0f

/* After triggering a note, ignore further triggers from the same
 * sensor for this long. Prevents the swipe's settling motion from
 * producing several notes. */
#define RETRIGGER_LOCKOUT_US    120000   /* 120 ms */
#define AUTO_REARM_US           RETRIGGER_LOCKOUT_US
#define READING_STALE_US        100000   /* >100 ms gap = previous state stale */

/* ------------------------------------------------------------------ */
/* Per-sensor state                                                    */
/* ------------------------------------------------------------------ */

/* I2C addresses each ToF gets after re-addressing. The default 0x29
 * is reserved for whichever sensor still has XSHUT low. */
static const uint8_t SENSOR_ADDRS[NUM_TOF_SENSORS] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37
};

/* C-major scale, sensor 0 = root (1.0×), sensor 7 = octave (2.0×).
 * Speed = 2 ** (semitones / 12).  C D E F G A B C = 0,2,4,5,7,9,11,12. */
static const float SCALE_SPEED[NUM_TOF_SENSORS] = {
    1.0f,                       /* C   (root)    */
    1.122462048309373f,         /* D   (+2 st)   */
    1.259921049894873f,         /* E   (+4 st)   */
    1.334839854170034f,         /* F   (+5 st)   */
    1.498307076876682f,         /* G   (+7 st)   */
    1.681792830507429f,         /* A   (+9 st)   */
    1.887748625363386f,         /* B   (+11 st)  */
    2.0f,                       /* C   (+octave) */
};

/* MODE_SYNTH: same C-major degrees as SCALE_SPEED — MIDI notes for synth_voice. */
static const uint8_t SYNTH_C_MAJOR_MIDI[NUM_TOF_SENSORS] = {
    60, 62, 64, 65, 67, 69, 71, 72,   /* C4 .. C5 */
};

/* Hand distance (mm) → synth volume: closer = louder. */
#define SYNTH_VOL_MM_CLOSE      140   /* near sensor → full volume */
#define SYNTH_VOL_MM_FAR        380   /* farther → silence */
/* Drop hand tracking if this sensor hasn't produced a valid range recently. */
#define SYNTH_MAX_READING_AGE_US 75000

struct swipe_state_t {
    uint16_t prev_dist_mm;
    int64_t  prev_time_us;
    float    smoothed_speed_mmps;   /* +ve = approaching */
    bool     armed;                 /* hand has been above ARM threshold */
    int64_t  last_trigger_us;
};

static VL53L0X       *s_sensors[NUM_TOF_SENSORS]  = {nullptr};
static swipe_state_t  s_swipe[NUM_TOF_SENSORS]    = {};
static std::atomic<uint16_t> s_last_dist_mm[NUM_TOF_SENSORS];
static std::atomic<int64_t>  s_last_update_us[NUM_TOF_SENSORS];

static bool s_initialised = false;
static TaskHandle_t s_task_handle = nullptr;

struct gliss_state_t {
    int     last_idx;
    int64_t last_cross_us;
    int8_t  dir;               /* -1 left, +1 right, 0 unknown */
    int     sensors_in_run;    /* count of sensors crossed in current run */
    bool    active;            /* run has met speed + length threshold */
};
static gliss_state_t s_gliss = {
    .last_idx = -1,
    .last_cross_us = 0,
    .dir = 0,
    .sensors_in_run = 0,
    .active = false,
};

/* ------------------------------------------------------------------ */
/* 74HC595 shift register                                              */
/* ------------------------------------------------------------------ */

static void shift_init_gpio(void)
{
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << SHIFT_SHCP_PIN) |
                      (1ULL << SHIFT_STPC_PIN) |
                      (1ULL << SHIFT_DS_PIN);
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(SHIFT_SHCP_PIN, 0);
    gpio_set_level(SHIFT_STPC_PIN, 0);
    gpio_set_level(SHIFT_DS_PIN, 0);
}

/* Shift `bitmask` out MSB-first (bit 7 -> Q7, bit 0 -> Q0). */
static void shift_write(uint8_t bitmask)
{
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(SHIFT_SHCP_PIN, 0);
        gpio_set_level(SHIFT_DS_PIN, (bitmask >> i) & 0x01);
        gpio_set_level(SHIFT_SHCP_PIN, 1);
    }
    gpio_set_level(SHIFT_STPC_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(SHIFT_STPC_PIN, 0);
}

/* ------------------------------------------------------------------ */
/* Swipe detection                                                     */
/* ------------------------------------------------------------------ */

static inline uint8_t velocity_from_speed(float speed_mmps)
{
    if (speed_mmps <= MIN_SWIPE_SPEED_MMPS) return 0;
    float t = (speed_mmps - MIN_SWIPE_SPEED_MMPS) /
              (MAX_SWIPE_SPEED_MMPS - MIN_SWIPE_SPEED_MMPS);
    if (t > 1.0f) t = 1.0f;
    /* Wider dynamic range at the low end:
     * - lower floor (8 instead of 40) so very soft swipes are quieter
     * - square-law curve so most of the velocity range is reserved for
     *   medium/fast attacks rather than bunching up near loud values. */
    float shaped = t * t;
    int v = 8 + (int)(shaped * 247.0f);
    if (v > 255) v = 255;
    return (uint8_t)v;
}

static inline uint8_t gliss_velocity_floor(void)
{
    return 42;
}

static bool update_gliss_state_on_crossing(int idx, int64_t now_us)
{
    /* New run if first crossing or timeout. */
    if (s_gliss.last_idx < 0 ||
        (now_us - s_gliss.last_cross_us) > GLISS_RUN_TIMEOUT_US) {
        s_gliss.last_idx = idx;
        s_gliss.last_cross_us = now_us;
        s_gliss.dir = 0;
        s_gliss.sensors_in_run = 1;
        s_gliss.active = false;
        return false;
    }

    int d_idx = idx - s_gliss.last_idx;
    if (d_idx == 0) {
        s_gliss.last_cross_us = now_us;
        return s_gliss.active;
    }

    int step = (d_idx > 0) ? d_idx : -d_idx;
    int8_t dir = (d_idx > 0) ? 1 : -1;
    float dt_s = (now_us - s_gliss.last_cross_us) / 1.0e6f;
    if (dt_s < 0.001f) dt_s = 0.001f;
    float lateral_mmps = (step * SENSOR_PITCH_MM) / dt_s;

    bool contiguous = (step == 1);
    bool same_dir = (s_gliss.dir == 0 || dir == s_gliss.dir);
    bool fast_enough = (lateral_mmps >= GLISS_MIN_LATERAL_MMPS);

    if (contiguous && same_dir && fast_enough) {
        s_gliss.sensors_in_run++;
        s_gliss.dir = dir;
        if (s_gliss.sensors_in_run >= GLISS_MIN_SENSORS) {
            s_gliss.active = true;
        }
    } else {
        /* Restart run at this sensor crossing. */
        s_gliss.sensors_in_run = 1;
        s_gliss.dir = 0;
        s_gliss.active = false;
    }

    s_gliss.last_idx = idx;
    s_gliss.last_cross_us = now_us;
    return s_gliss.active;
}

/* --- MODE_SYNTH loop recording: sustained notes (start, pitch, vel, duration) --- */
static bool     s_synrec_active = false;
static uint32_t s_synrec_start  = 0;
static uint8_t  s_synrec_midi   = 60;
static uint8_t  s_synrec_vel    = 0;
static int      s_synrec_track  = 0;
static bool     s_synrec_first_take = true;
static bool     s_synrec_was_rec = false;
static uint32_t s_synrec_last_cap = 0;

static uint32_t synth_seg_duration_us(uint32_t start, uint32_t end,
                                      uint32_t loop_len, bool first_take)
{
    if (first_take || loop_len == 0) {
        return (end >= start) ? (end - start) : 0;
    }
    if (end >= start) {
        return end - start;
    }
    return (loop_len - start) + end;
}

static void synth_commit_segment(uint32_t end_cap, uint32_t loop_len, bool first_take)
{
    if (!s_synrec_active) {
        return;
    }
    uint32_t dur =
        synth_seg_duration_us(s_synrec_start, end_cap, loop_len, first_take);
    s_synrec_active = false;
    if (dur < 1000 || s_synrec_vel < 1) {
        return;
    }

    note_event_t e = {};
    e.slot         = s_synrec_midi;
    e.velocity     = s_synrec_vel;
    e.speed        = 1.0f;
    e.loop         = false;
    e.source       = NOTE_SOURCE_SYNTH_NOTE;
    e.track        = (uint8_t)s_synrec_track;
    e.duration_us  = dur;
    e.tape_time_us = s_synrec_start;
    note_event_post_full(&e);
}

/**
 * Live ToF synth only in MODE_SYNTH. Synth tape on any track replays whenever
 * that track was recorded in MODE_SYNTH, independent of the current global mode.
 */
static void synth_tof_apply_gesture(int64_t now_us)
{
    if (!shared_state_lock()) {
        return;
    }

    int tr = (int)g_state.active_track;
    if (tr < 0 || tr >= NUM_TRACKS) {
        tr = 0;
    }

    bool     rec_now  = g_state.is_recording;
    uint32_t loop_len = loop_recorder_loop_length_us();
    uint32_t cap      = 0;

    if (rec_now) {
        cap = loop_recorder_capture_time_us();
    }

    if (s_synrec_was_rec && !rec_now && s_synrec_active) {
        synth_commit_segment(s_synrec_last_cap, loop_len, s_synrec_first_take);
    }
    s_synrec_was_rec = rec_now;

    if (g_state.mode != MODE_SYNTH) {
        s_synrec_active = false;
    }

    int      best_idx = -1;
    uint16_t best_d   = UINT16_MAX;

    if (g_state.mode == MODE_SYNTH) {
        for (int i = 0; i < NUM_TOF_SENSORS; i++) {
            int64_t t_up = s_last_update_us[i].load();
            if (t_up <= 0 || (now_us - t_up) > SYNTH_MAX_READING_AGE_US) {
                continue;
            }
            uint16_t d = s_last_dist_mm[i].load();
            if (d == 0 || d > DIST_RANGE_MAX) {
                continue;
            }
            if (d < 40) {
                continue;
            }
            if (d < best_d) {
                best_d = d;
                best_idx = i;
            }
        }
    }

    const bool live_hand = (g_state.mode == MODE_SYNTH && best_idx >= 0);
    uint32_t   ph        = loop_recorder_playhead_us();

    for (int t = 0; t < NUM_TRACKS; t++) {
        g_state.tracks[t].pitch_bend = g_state.master_detune_sem * 0.5f;
    }

    if (live_hand) {
        track_params_t *tp = &g_state.tracks[tr];
        uint8_t         midi = SYNTH_C_MAJOR_MIDI[best_idx];
        tp->pitch           = midi;

        float t = ((float)best_d - (float)SYNTH_VOL_MM_CLOSE) /
                  (float)(SYNTH_VOL_MM_FAR - SYNTH_VOL_MM_CLOSE);
        if (t < 0.0f) {
            t = 0.0f;
        }
        if (t > 1.0f) {
            t = 1.0f;
        }
        tp->volume = 1.0f - t;

        int vi = (int)(tp->volume * 255.0f);
        if (vi < 0) {
            vi = 0;
        }
        if (vi > 255) {
            vi = 255;
        }
        uint8_t vel8 = (uint8_t)vi;

        if (rec_now) {
            if (!s_synrec_active) {
                s_synrec_active     = true;
                s_synrec_start      = cap;
                s_synrec_midi       = midi;
                s_synrec_vel        = vel8;
                s_synrec_track      = tr;
                s_synrec_first_take = (loop_len == 0);
            } else if (midi != s_synrec_midi) {
                synth_commit_segment(cap, loop_len, s_synrec_first_take);
                s_synrec_active     = true;
                s_synrec_start      = cap;
                s_synrec_midi       = midi;
                s_synrec_vel        = vel8;
                s_synrec_track      = tr;
                s_synrec_first_take = (loop_len == 0);
            }
        }
    } else if (g_state.mode == MODE_SYNTH && rec_now && s_synrec_active) {
        synth_commit_segment(cap, loop_len, s_synrec_first_take);
    }

    if (rec_now) {
        s_synrec_last_cap = cap;
    }

    uint8_t pm = 60;
    float   pv = 0.0f;

    for (int t = 0; t < NUM_TRACKS; t++) {
        if (live_hand && t == tr) {
            continue;
        }
        if (!loop_recorder_track_recorded_as_synth(t)) {
            g_state.tracks[t].volume = 0.0f;
            continue;
        }
        if (loop_len == 0) {
            g_state.tracks[t].volume = 0.0f;
            continue;
        }
        if (loop_recorder_synth_playback_at(t, ph, &pm, &pv)) {
            g_state.tracks[t].pitch  = pm;
            g_state.tracks[t].volume = pv;
        } else {
            g_state.tracks[t].volume = 0.0f;
        }
    }

    shared_state_unlock();
}

/** Update per-sensor state with one new reading and (maybe) post a
 *  note event. Returns true if a note was triggered. */
static bool process_reading(int idx, uint16_t distance_mm, int64_t now_us)
{
    swipe_state_t &st = s_swipe[idx];

    /* Nothing in front of the sensor */
    if (distance_mm == 0 || distance_mm > DIST_RANGE_MAX) {
        st.prev_dist_mm = DIST_RANGE_MAX;
        st.prev_time_us = now_us;
        st.smoothed_speed_mmps = 0.0f;
        st.armed = true;          /* clearly above ARM */
        return false;
    }

    /* ToF → sampler only in SAMPLE mode (instrument swipes). SYNTH uses the
     * continuous path in synth_tof_apply_gesture(); DRUMS and other modes
     * use buttons / transport only — no ToF sample hits. */
    if (g_state.mode != MODE_SAMPLE) {
        st.prev_dist_mm = distance_mm;
        st.prev_time_us = now_us;
        return false;
    }

    /* Compute instantaneous approach speed */
    float dt_s = (now_us - st.prev_time_us) / 1.0e6f;
    if (dt_s < 0.001f) dt_s = 0.001f;

    /* If we haven't seen a valid reading on this sensor for a while, the
     * sensor's "previous distance" is effectively meaningless: the hand
     * left, the VL53L0X reported RangeStatus != 0 for far/no-object, and
     * we never updated state. Treat the previous state as "no hand" so
     * the next approach produces a clean crossing edge. Zero the speed
     * estimate so this stale-recovery reading can't spoof a high-velocity
     * strike. */
    bool stale = (now_us - st.prev_time_us) > READING_STALE_US;
    if (stale) {
        st.prev_dist_mm = DIST_RANGE_MAX;
        st.smoothed_speed_mmps = 0.0f;
    } else {
        float inst_speed =
            (float)((int)st.prev_dist_mm - (int)distance_mm) / dt_s;
        /* Light EMA so a single noisy frame doesn't blow the trigger */
        st.smoothed_speed_mmps =
            0.55f * st.smoothed_speed_mmps + 0.45f * inst_speed;
    }

    /* Re-arm when the hand pulls away (strike behavior). */
    if (distance_mm >= DIST_ARM_MM) {
        st.armed = true;
    }
    /* Also auto re-arm after lockout so repeated swipes don't require
     * a large hand lift between passes. */
    if (!st.armed && (now_us - st.last_trigger_us) > AUTO_REARM_US) {
        st.armed = true;
    }

    bool fired = false;

    /* A sensor "crossing" event is the hand entering this sensor's
     * strike zone. We use this to track lateral gliss speed. */
    bool crossing =
        (st.prev_dist_mm > DIST_TRIGGER_MM) &&
        (distance_mm <= DIST_TRIGGER_MM);

    bool gliss_active_now = false;
    if (crossing) {
        gliss_active_now = update_gliss_state_on_crossing(idx, now_us);
    }

    /* Two trigger paths:
     *  1) strike trigger: threshold crossing + enough approach speed + armed.
     *  2) gliss trigger: crossing while a high-speed 3+ sensor swipe is active.
     *     (No re-arm requirement.) */
    bool strike_crossing =
        crossing &&
        (st.smoothed_speed_mmps > MIN_SWIPE_SPEED_MMPS);

    bool gliss_crossing = crossing && gliss_active_now;

    if ((((st.armed && strike_crossing) || gliss_crossing)) &&
        (now_us - st.last_trigger_us) > RETRIGGER_LOCKOUT_US) {

        uint8_t velocity = velocity_from_speed(st.smoothed_speed_mmps);
        if (velocity == 0 && gliss_crossing) {
            velocity = gliss_velocity_floor();
        }
        if (velocity > 0) {
            /* Lock-free read of currently active instrument. We don't
             * take g_state_mutex here — `active_instrument` is a single
             * 32-bit aligned enum value, so a torn read is impossible
             * on Xtensa. Worst case: we use last-cycle's instrument. */
            instrument_t inst = g_state.active_instrument;
            uint8_t slot = instrument_to_slot(inst);
            float   speed = SCALE_SPEED[idx];

            note_event_post(slot, velocity, speed,
                            /*loop=*/false,
                            /*source=*/(int8_t)idx,
                            /*track=*/0);
            st.armed = false;
            st.last_trigger_us = now_us;
            fired = true;

            ESP_LOGD(TAG, "ToF %d -> slot %d, vel %d, speed %.3f",
                     idx, slot, velocity, speed);
        }
    }

    st.prev_dist_mm = distance_mm;
    st.prev_time_us = now_us;
    return fired;
}

/* ------------------------------------------------------------------ */
/* Polling task                                                        */
/* ------------------------------------------------------------------ */

static void sensor_polling_task(void *)
{
    ESP_LOGI(TAG, "Polling task running on core %d, %d sensors",
             xPortGetCoreID(), NUM_TOF_SENSORS);

    VL53L0X_RangingMeasurementData_t data;

    while (true) {
        int64_t now_us = esp_timer_get_time();

        for (int i = 0; i < NUM_TOF_SENSORS; i++) {
            VL53L0X *s = s_sensors[i];
            if (s == nullptr) continue;

            VL53L0X_Dev_t *dev = s->get_dev();
            uint8_t ready = 0;
            VL53L0X_GetMeasurementDataReady(dev, &ready);
            if (!ready) continue;

            if (VL53L0X_GetRangingMeasurementData(dev, &data) == VL53L0X_ERROR_NONE
                && data.RangeStatus == 0) {
                uint16_t mm = data.RangeMilliMeter;
                s_last_dist_mm[i].store(mm);
                s_last_update_us[i].store(now_us);
                process_reading(i, mm, now_us);
            } else {
                /* Invalidate stale range so MODE_SYNTH (and LEDs) don't stick on
                 * the last in-range value after the object leaves the FoV. */
                s_last_dist_mm[i].store(0);
                s_last_update_us[i].store(now_us);
            }

            VL53L0X_ClearInterruptMask(dev, 0);
        }

        synth_tof_apply_gesture(esp_timer_get_time());

        panel_input_poll();

        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

bool sensor_task_init(void)
{
    if (s_initialised) {
        ESP_LOGW(TAG, "sensor_task_init() called twice — ignoring");
        return true;
    }

    /* Per-sensor swipe state */
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        s_swipe[i].prev_dist_mm        = DIST_RANGE_MAX;
        s_swipe[i].prev_time_us        = esp_timer_get_time();
        s_swipe[i].smoothed_speed_mmps = 0.0f;
        s_swipe[i].armed               = true;
        s_swipe[i].last_trigger_us     = 0;
        s_last_dist_mm[i].store(0);
        s_last_update_us[i].store(0);
    }

    /* 74HC595 setup, all XSHUT lines low (sensors held in reset). */
    shift_init_gpio();
    shift_write(0x00);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* I2C bus init via the VL53L0X helper. The wrapper installs the
     * legacy i2c master driver on the requested port and sets up the
     * default 0x29 device. Lennart's working code uses this exact
     * sequence. */
    VL53L0X bus_helper(I2C_NUM_0);
    bus_helper.i2cMasterInit(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

    /* Bring up sensors one at a time. While bit `i` of the shift
     * register is high, sensor i has XSHUT released; all others stay
     * in reset, so only one device responds at the default 0x29
     * address. We then move it to its final unique address. */
    uint8_t mask = 0;
    int initialised = 0;
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        mask |= (uint8_t)(1u << i);
        shift_write(mask);
        vTaskDelay(pdMS_TO_TICKS(60));     /* boot time + readiness */

        s_sensors[i] = new VL53L0X(I2C_NUM_0, GPIO_NUM_MAX, GPIO_NUM_MAX);
        if (!s_sensors[i]->init()) {
            ESP_LOGE(TAG, "ToF %d failed to init", i);
            delete s_sensors[i];
            s_sensors[i] = nullptr;
            led_task_boot_set_sensor_ready(i, false);
            continue;
        }

        s_sensors[i]->setDeviceAddress(SENSOR_ADDRS[i]);
        vTaskDelay(pdMS_TO_TICKS(15));
        s_sensors[i]->setTimingBudget(SENSOR_TIMING_US);

        VL53L0X_Dev_t *dev = s_sensors[i]->get_dev();
        VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        VL53L0X_StartMeasurement(dev);

        ESP_LOGI(TAG, "ToF %d ready @ 0x%02X (scale step %.3f)",
                 i, SENSOR_ADDRS[i], SCALE_SPEED[i]);
        led_task_boot_set_sensor_ready(i, true);
        initialised++;
    }

    if (initialised == 0) {
        ESP_LOGE(TAG, "No sensors came up — gesture input is disabled");
        return false;
    }

    ESP_LOGI(TAG, "%d / %d sensors online", initialised, NUM_TOF_SENSORS);
    s_initialised = true;
    return true;
}

void sensor_task_start(void)
{
    if (!s_initialised) {
        ESP_LOGE(TAG, "sensor_task_start() called before _init() succeeded");
        return;
    }
    if (s_task_handle != nullptr) return;

    /* Pin to Core 1 with priority 5 (well below the audio task at
     * configMAX_PRIORITIES-1, so polling never preempts audio). */
    xTaskCreatePinnedToCore(sensor_polling_task,
                            "sensor", 4096, nullptr,
                            5, &s_task_handle, 1);
}

uint16_t sensor_task_last_distance(int idx)
{
    if (idx < 0 || idx >= NUM_TOF_SENSORS) return 0;
    return s_last_dist_mm[idx].load();
}

int32_t sensor_task_last_age_ms(int idx)
{
    if (idx < 0 || idx >= NUM_TOF_SENSORS) return INT32_MAX;
    int64_t last = s_last_update_us[idx].load();
    if (last <= 0) return INT32_MAX;

    int64_t age_ms = (esp_timer_get_time() - last) / 1000;
    if (age_ms < 0) age_ms = 0;
    if (age_ms > INT32_MAX) age_ms = INT32_MAX;
    return (int32_t)age_ms;
}
