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
 * Threading: this whole file is one producer task; it only writes the
 * shared state through the note-event queue and reads g_state via a
 * snapshot, so we don't touch g_state_mutex inside the hot polling
 * loop.
 */

#include "sensor_task.h"
#include "shared_state.h"

#include <math.h>
#include <string.h>
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
#define DIST_ARM_MM         350      /* hand must rise above this to re-arm */
#define DIST_TRIGGER_MM     200      /* crossing below this fires a note */
#define DIST_RANGE_MAX      1200     /* anything farther = "no hand" */

/* Velocity → MIDI velocity (sample volume) mapping.
 * Approach speed in mm/sec:
 *   100  mm/s  -> roughly noise floor, ignored
 *   400  mm/s  -> mezzo-piano (~80)
 *   1500 mm/s  -> fortissimo (255)
 * Anything above that clips to 255. */
#define MIN_SWIPE_SPEED_MMPS    150.0f
#define MAX_SWIPE_SPEED_MMPS    1500.0f

/* After triggering a note, ignore further triggers from the same
 * sensor for this long. Prevents the swipe's settling motion from
 * producing several notes. */
#define RETRIGGER_LOCKOUT_US    120000   /* 120 ms */

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

static bool s_initialised = false;
static TaskHandle_t s_task_handle = nullptr;

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
    /* Map [0..1] to MIDI velocity [40..255] so even soft swipes are
     * audible while leaving headroom for hard ones. */
    int v = 40 + (int)(t * 215.0f);
    if (v > 255) v = 255;
    return (uint8_t)v;
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

    /* Compute instantaneous approach speed */
    float dt_s = (now_us - st.prev_time_us) / 1.0e6f;
    if (dt_s < 0.001f) dt_s = 0.001f;
    float inst_speed = (float)((int)st.prev_dist_mm - (int)distance_mm) / dt_s;

    /* Light EMA so a single noisy frame doesn't blow the trigger */
    st.smoothed_speed_mmps =
        0.55f * st.smoothed_speed_mmps + 0.45f * inst_speed;

    /* Re-arm when the hand pulls away */
    if (distance_mm >= DIST_ARM_MM) {
        st.armed = true;
    }

    bool fired = false;

    /* Edge: crossing below TRIGGER threshold while armed */
    if (st.armed &&
        st.prev_dist_mm > DIST_TRIGGER_MM &&
        distance_mm    <= DIST_TRIGGER_MM &&
        st.smoothed_speed_mmps > MIN_SWIPE_SPEED_MMPS &&
        (now_us - st.last_trigger_us) > RETRIGGER_LOCKOUT_US) {

        uint8_t velocity = velocity_from_speed(st.smoothed_speed_mmps);
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
                            /*source=*/(int8_t)idx);
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
                process_reading(i, mm, now_us);
            }

            VL53L0X_ClearInterruptMask(dev, 0);
        }

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
