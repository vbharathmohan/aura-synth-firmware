/**
 * panel_input.c — 3×3 matrix + ADC sliders for Aura-Synth INTEGRATION_MODE.
 *
 * =============================================================================
 * FEATHER ESP32 V2 — WIRING (GPIOs chosen to avoid I2S / I2C / 595 / XSMT / LED)
 * =============================================================================
 *
 * MATRIX (each button connects ROW to COL when pressed; no common ground):
 *
 *              COL0           COL1           COL2
 *           GPIO 7         GPIO 21        GPIO 19
 *           (RX)           (MISO)         (MOSI)
 *              |              |              |
 *   ROW0(15)--[BTN0]--------[BTN1]---------[BTN2]--
 *              |              |              |
 *   ROW1(32)--[BTN3]--------[BTN4]---------[BTN5]--
 *              |              |              |
 *   ROW2(5)---[BTN6]--------[BTN7]---------[BTN8]--
 *
 * Physical matrix currently wired as:
 *   Row 0: kick / clap / clear
 *   Row 1: hihat / track cycle / play-pause
 *   Row 2: snare / cycle mode / record-stop
 *
 * Reconciled logical actions (desired UX) mapped onto those physical buttons:
 *   BTN0 (R0C0 kick)        -> kick (DRUMS only)
 *   BTN6 (R2C0 snare)       -> snare (DRUMS only)
 *   BTN3 (R1C0 hihat)       -> hi-hat (DRUMS only)
 *   BTN1 (R0C1 clap)        -> clap (DRUMS) / cycle instrument (SAMPLE)
 *   BTN7 (R2C1 cycle mode)  -> cycle global mode (sample -> drums -> synth)
 *   BTN4 (R1C1 track cycle) -> cycle active track
 *   BTN2 (R0C2 clear)       -> clear track
 *   BTN8 (R2C2 record-stop) -> record toggle
 *   BTN5 (R1C2 play-pause)  -> play/pause
 *
 *   Drum-pad buttons (BTN0/1/2 and BTN3-as-clap) only fire when the global
 *   mode is MODE_DRUMS. BTN3 cycles instruments when the global mode is
 *   MODE_SAMPLE; in MODE_SYNTH it does nothing. Transport buttons (Row 2,
 *   BTN4, BTN5) are always active.
 *
 *   ROW pins: OUTPUT, idle HIGH; scan pulls one row LOW at a time.
 *   COL pins: INPUT + internal pull-up; LOW = button ties col to active row.
 *
 * ANALOG (middle wiper → GPIO; ends → 3.3 V and GND):
 *   Slider 1 (GPIO 34, A2): master volume 0 .. 1
 *   Slider 2 (GPIO 39, A3): pitch bend -1 .. +1 semitone (center = in tune)
 *   Dial — filter (GPIO 36, A4): master LPF cutoff (see MASTER_FILTER_*)
 *   Dial — LFO    (GPIO 37):     0 .. 10 Hz
 *
 *   Master reverb / delay is **forced off** in firmware (master_reverb = 0).
 *   A reverb pot on GPIO 35 is not read until hardware is wired and this is
 *   re-enabled. (GPIO 35 is often VBAT on Feather; do not sample it blindly.)
 *
 * RESERVED ELSEWHERE (do not use for matrix):
 *   4,25,26 = I2S; 20,22 = I2C ToF; 12,13,14 = 74HC595; 27 = XSMT; 33 = WS2812
 *
 * NOTE: COL0 is UART RX (GPIO 7); COL1 is SPI MISO (GPIO 21); COL2 is MOSI (19).
 * Do not use GPIO 8 (TX) for the matrix — reserved / avoided on this build.
 */

#include "panel_input.h"
#include "shared_state.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

static const char *TAG = "panel_input";

/* Row / column GPIOs — must not clash with sensor_task / i2s / led_task */
#define ROW0_PIN    GPIO_NUM_15
#define ROW1_PIN    GPIO_NUM_32
#define ROW2_PIN    GPIO_NUM_5

#define COL0_PIN    GPIO_NUM_7
#define COL1_PIN    GPIO_NUM_21
#define COL2_PIN    GPIO_NUM_19

#define SLIDER_VOL_ADC_CHANNEL    ADC_CHANNEL_6   /* GPIO 34 — volume */
#define SLIDER_BEND_ADC_CHANNEL   ADC_CHANNEL_3   /* GPIO 39 — ±1 semitone bend */
#define DIAL_FILTER_ADC_CHANNEL   ADC_CHANNEL_0   /* GPIO 36, A4 — filter */
#define DIAL_LFO_ADC_CHANNEL      ADC_CHANNEL_1   /* GPIO 37 — LFO */
/* #define DIAL_REVERB_ADC_CHANNEL ADC_CHANNEL_7 */ /* GPIO 35 — reserved; reverb off */

#define NUM_ROWS    3
#define NUM_COLS    3
#define NUM_BUTTONS (NUM_ROWS * NUM_COLS)

#define PANEL_SCAN_PERIOD_US    10000   /* 100 Hz */
#define DEBOUNCE_THRESHOLD      3

#define MASTER_FILTER_MIN_HZ    300.0f
#define MASTER_FILTER_MAX_HZ    6000.0f

static const gpio_num_t ROW_PINS[NUM_ROWS] = { ROW0_PIN, ROW1_PIN, ROW2_PIN };
static const gpio_num_t COL_PINS[NUM_COLS] = { COL0_PIN, COL1_PIN, COL2_PIN };

/* Drum-pad mapping for the new matrix layout.
 *   Row 0 buttons (BTN0/1/2) + BTN3 (when in MODE_DRUMS) form a 4-pad bank.
 *   Index within this array is the button slot, NOT the matrix btn_idx. */
#define DRUM_PAD_KICK    0
#define DRUM_PAD_SNARE   1
#define DRUM_PAD_HIHAT   2
#define DRUM_PAD_CLAP    3

static const uint8_t s_drum_pad_slot[NUM_DRUM_PADS] = {
    SAMPLE_SLOT_KICK,
    SAMPLE_SLOT_SNARE,
    SAMPLE_SLOT_HIHAT,
    SAMPLE_SLOT_CLAP,
};

/* SAMPLE-mode instrument cycle order for the physical clap button (BTN1). */
static const instrument_t s_instrument_cycle[NUM_INSTRUMENTS] = {
    INST_PIANO, INST_STEEL_DRUM, INST_TRUMPET, INST_808_BASS,
};

static bool     btn_state[NUM_BUTTONS];
static bool     btn_raw[NUM_BUTTONS];
static int      btn_debounce[NUM_BUTTONS];
static bool     btn_prev[NUM_BUTTONS];

static adc_oneshot_unit_handle_t s_adc;
static bool     s_inited;
static int64_t  s_last_scan_us;

static float    s_vol_ema;     /* slider1 → master volume */
static float    s_bend_ema;    /* slider2 → pitch bend (normalized; 0.5 = center) */
static float    s_filt_ema;    /* dial GPIO36 → LPF cutoff */
static float    s_lfo_ema;     /* dial → LFO Hz */

static void matrix_gpio_init(void)
{
    for (int r = 0; r < NUM_ROWS; r++) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << ROW_PINS[r]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
        gpio_set_level(ROW_PINS[r], 1);
    }

    for (int c = 0; c < NUM_COLS; c++) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << COL_PINS[c]),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
    }

    ESP_LOGI(TAG, "Matrix: rows GPIO %d,%d,%d | cols GPIO %d,%d,%d",
             ROW0_PIN, ROW1_PIN, ROW2_PIN, COL0_PIN, COL1_PIN, COL2_PIN);
}

static void matrix_scan(void)
{
    for (int r = 0; r < NUM_ROWS; r++) {
        gpio_set_level(ROW_PINS[r], 0);
        esp_rom_delay_us(10);

        for (int c = 0; c < NUM_COLS; c++) {
            int btn_idx = r * NUM_COLS + c;
            btn_raw[btn_idx] = (gpio_get_level(COL_PINS[c]) == 0);
        }

        gpio_set_level(ROW_PINS[r], 1);
    }

    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (btn_raw[i] == btn_state[i]) {
            btn_debounce[i] = 0;
        } else {
            btn_debounce[i]++;
            if (btn_debounce[i] >= DEBOUNCE_THRESHOLD) {
                btn_state[i] = btn_raw[i];
                btn_debounce[i] = 0;
            }
        }
    }
}

static void adc_hw_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, SLIDER_VOL_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, SLIDER_BEND_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, DIAL_FILTER_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, DIAL_LFO_ADC_CHANNEL, &chan_cfg));

    ESP_LOGI(TAG,
             "ADC: vol=GPIO34 ch%d | bend=GPIO39 ch%d | filt=GPIO36 ch%d | lfo=GPIO37 ch%d",
             SLIDER_VOL_ADC_CHANNEL, SLIDER_BEND_ADC_CHANNEL, DIAL_FILTER_ADC_CHANNEL,
             DIAL_LFO_ADC_CHANNEL);
}

/* Cycle order for physical cycle-mode button (BTN7): SAMPLE -> DRUMS -> SYNTH -> SAMPLE. */
static instrument_mode_t next_global_mode(instrument_mode_t cur)
{
    switch (cur) {
    case MODE_SAMPLE: return MODE_DRUMS;
    case MODE_DRUMS:  return MODE_SYNTH;
    case MODE_SYNTH:  return MODE_SAMPLE;
    default:          return MODE_SAMPLE;
    }
}

static const char *mode_name(instrument_mode_t m)
{
    switch (m) {
    case MODE_SAMPLE: return "SAMPLE";
    case MODE_DRUMS:  return "DRUMS";
    case MODE_SYNTH:  return "SYNTH";
    case MODE_PIANO:  return "PIANO";
    case MODE_FX:     return "FX";
    default:          return "?";
    }
}

static void fire_drum_pad(int pad_idx)
{
    if (pad_idx < 0 || pad_idx >= NUM_DRUM_PADS) return;
    note_event_post(s_drum_pad_slot[pad_idx],
                    /*velocity=*/200, /*speed=*/1.0f, /*loop=*/false,
                    /*source=*/(int8_t)(10 + pad_idx),
                    /*track=*/0);
}

static void handle_button_edges(void)
{
    for (int i = 0; i < NUM_BUTTONS; i++) {
        bool pressed = btn_state[i];
        bool edge    = pressed && !btn_prev[i];
        btn_prev[i]  = pressed;

        if (!edge) {
            continue;
        }

        instrument_mode_t mode = g_state.mode;

        switch (i) {
        case 0: /* Row0 Col0: Kick */
            if (mode == MODE_DRUMS) {
                fire_drum_pad(DRUM_PAD_KICK);
                ESP_LOGI(TAG, "Drum: KICK");
            }
            break;

        case 2: /* Row0 Col1: Clap / cycle instrument */
            if (mode == MODE_DRUMS) {
                fire_drum_pad(DRUM_PAD_CLAP);
                ESP_LOGI(TAG, "Drum: CLAP");
            } else if (mode == MODE_SAMPLE) {
                instrument_t next_inst = INST_PIANO;
                if (shared_state_lock()) {
                    int cur = (int)g_state.active_instrument;
                    next_inst = s_instrument_cycle[(cur + 1) % NUM_INSTRUMENTS];
                    g_state.active_instrument = next_inst;
                    shared_state_unlock();
                }
                ESP_LOGI(TAG, "Instrument -> %d", (int)next_inst);
            }
            break;

        case 1: /* Row0 Col2: Clear track */
            if (shared_state_lock()) {
                g_state.clear_pressed = true;
                shared_state_unlock();
            }
            ESP_LOGI(TAG, "Transport: CLEAR");
            break;

        case 6: /* Row1 Col0: Hihat */
            if (mode == MODE_DRUMS) {
                fire_drum_pad(DRUM_PAD_HIHAT);
                ESP_LOGI(TAG, "Drum: HIHAT");
            }
            break;

        case 8: /* Row1 Col1: Cycle track */
            {
                int new_track = 0;
                if (shared_state_lock()) {
                    new_track = (g_state.active_track + 1) % NUM_TRACKS;
                    g_state.active_track = (uint8_t)new_track;
                    shared_state_unlock();
                }
                ESP_LOGI(TAG, "Track -> %d", new_track);
            }
            break;

        case 7: /* Row1 Col2: Play/pause */
            if (shared_state_lock()) {
                g_state.play_pause_pressed = true;
                shared_state_unlock();
            }
            ESP_LOGI(TAG, "Transport: PLAY/PAUSE");
            break;

        case 3: /* Row2 Col0: Snare */
            if (mode == MODE_DRUMS) {
                fire_drum_pad(DRUM_PAD_SNARE);
                ESP_LOGI(TAG, "Drum: SNARE");
            }
            break;

        case 5: /* Row2 Col1: Cycle mode */
            {
                instrument_mode_t new_mode = MODE_SAMPLE;
                if (shared_state_lock()) {
                    new_mode = next_global_mode(g_state.mode);
                    g_state.mode = new_mode;
                    shared_state_unlock();
                }
                ESP_LOGI(TAG, "Mode -> %s", mode_name(new_mode));
            }
            break;

        case 4: /* Row2 Col2: Record/stop recording */
            if (shared_state_lock()) {
                g_state.record_pressed = true;
                shared_state_unlock();
            }
            ESP_LOGI(TAG, "Transport: REC");
            break;

        default:
            break;
        }
    }
}

static void read_analog_and_apply(void)
{
    int rv = 0, rr = 0, rf = 0, rl = 0;
    if (adc_oneshot_read(s_adc, SLIDER_VOL_ADC_CHANNEL, &rv) != ESP_OK) {
        return;
    }
    if (adc_oneshot_read(s_adc, SLIDER_BEND_ADC_CHANNEL, &rr) != ESP_OK) {
        return;
    }
    if (adc_oneshot_read(s_adc, DIAL_FILTER_ADC_CHANNEL, &rf) != ESP_OK) {
        return;
    }
    if (adc_oneshot_read(s_adc, DIAL_LFO_ADC_CHANNEL, &rl) != ESP_OK) {
        return;
    }

    if (rv < 0) {
        rv = 0;
    }
    if (rr < 0) {
        rr = 0;
    }
    if (rf < 0) {
        rf = 0;
    }
    if (rl < 0) {
        rl = 0;
    }

    float nv = fminf(1.0f, (float)rv / 4095.0f);
    float nr = fminf(1.0f, (float)rr / 4095.0f);
    float nf = fminf(1.0f, (float)rf / 4095.0f);
    float nl = fminf(1.0f, (float)rl / 4095.0f);

    const float a = 0.25f;
    s_vol_ema  += (nv - s_vol_ema) * a;
    s_bend_ema += (nr - s_bend_ema) * a;
    s_filt_ema += (nf - s_filt_ema) * a;
    s_lfo_ema  += (nl - s_lfo_ema) * a;

    float vol = s_vol_ema;
    /* Slider2: pitch bend -1 .. +1 semitone (center = 0.5 normalized) */
    float bend_sem = (s_bend_ema - 0.5f) * 2.0f;
    float lfo_hz = s_lfo_ema * 10.0f;
    float f_hz   = MASTER_FILTER_MIN_HZ +
                   s_filt_ema * (MASTER_FILTER_MAX_HZ - MASTER_FILTER_MIN_HZ);

    if (shared_state_lock()) {
        g_state.master_volume         = vol;
        g_state.master_filter         = f_hz;
        g_state.master_playback_rate  = 1.0f;
        g_state.master_detune_sem     = bend_sem;
        g_state.master_lfo_hz         = lfo_hz;
        g_state.master_reverb         = 0.0f;
        g_state.master_delay_mix      = 0.0f; /* legacy; reverb path off */
        shared_state_unlock();
    }
}

void panel_input_init(void)
{
    if (s_inited) {
        return;
    }

    memset(btn_state, 0, sizeof(btn_state));
    memset(btn_raw, 0, sizeof(btn_raw));
    memset(btn_debounce, 0, sizeof(btn_debounce));
    memset(btn_prev, 0, sizeof(btn_prev));
    s_vol_ema  = 0.85f;
    s_bend_ema = 0.5f; /* pitch bend 0 semitones */
    s_filt_ema = 0.5f;
    s_lfo_ema  = 0.0f;
    s_last_scan_us = 0;

    matrix_gpio_init();
    adc_hw_init();
    s_inited = true;

    ESP_LOGI(TAG, "Panel ready (see file header for full GPIO map)");
}

void panel_input_poll(void)
{
    if (!s_inited) {
        return;
    }

    int64_t now = esp_timer_get_time();
    if (now - s_last_scan_us < PANEL_SCAN_PERIOD_US) {
        return;
    }
    s_last_scan_us = now;

    matrix_scan();
    handle_button_edges();
    read_analog_and_apply();
}
