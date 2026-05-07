/**
 * panel_input.c — 3×3 matrix + ADC sliders for Aura-Synth INTEGRATION_MODE.
 *
 * =============================================================================
 * FEATHER ESP32 V2 — WIRING (GPIOs chosen to avoid I2S / I2C / 595 / XSMT / LED)
 * =============================================================================
 *
 * MATRIX (each button connects ROW to COL when pressed; no common ground):
 *
 *              COL0          COL1          COL2
 *           GPIO 7        GPIO 21      GPIO 19
 *         (RX)          (MISO)       (MOSI)
 *              |             |             |
 *   ROW0(15)--[BTN0]-------[BTN1]-------[BTN2]--   Pad0 / Pad1 / Pad2
 *              |             |             |
 *   ROW1(32)--[BTN3]-------[BTN4]-------[BTN5]--   Pad3 / PadTog / Record
 *              |             |             |
 *   ROW2(5)---[BTN6]-------[BTN7]-------[BTN8]--   Clear / Play / TrkCycle
 *
 *   ROW pins: OUTPUT, idle HIGH; scan pulls one row LOW at a time.
 *   COL pins: INPUT + internal pull-up; LOW = button ties col to active row.
 *
 * ANALOG (middle wiper → GPIO; ends → 3.3 V and GND):
 *   Slider 1 (master volume):   GPIO 34  (A2)  ADC1_CH6
 *   Slider 2 (master filter):   GPIO 39  (A3)  ADC1_CH3
 *   Pot      (master delay mix): GPIO 36  (A4)  ADC1_CH0
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

#define SLIDER1_ADC_CHANNEL   ADC_CHANNEL_6   /* GPIO 34 */
#define SLIDER2_ADC_CHANNEL   ADC_CHANNEL_3   /* GPIO 39 */
#define POT_ADC_CHANNEL       ADC_CHANNEL_0   /* GPIO 36 */

#define NUM_ROWS    3
#define NUM_COLS    3
#define NUM_BUTTONS (NUM_ROWS * NUM_COLS)

#define PANEL_SCAN_PERIOD_US    10000   /* 100 Hz */
#define DEBOUNCE_THRESHOLD      3

#define MASTER_FILTER_MIN_HZ    300.0f
#define MASTER_FILTER_MAX_HZ    6000.0f

static const gpio_num_t ROW_PINS[NUM_ROWS] = { ROW0_PIN, ROW1_PIN, ROW2_PIN };
static const gpio_num_t COL_PINS[NUM_COLS] = { COL0_PIN, COL1_PIN, COL2_PIN };

static const uint8_t s_pad_drum_slot[NUM_DRUM_PADS] = {
    SAMPLE_SLOT_KICK,  SAMPLE_SLOT_HIHAT,
    SAMPLE_SLOT_CLAP,  SAMPLE_SLOT_SNARE,
};

static const instrument_t s_pad_instrument[NUM_DRUM_PADS] = {
    INST_PIANO, INST_STEEL_DRUM, INST_TRUMPET, INST_808_BASS,
};

static bool     btn_state[NUM_BUTTONS];
static bool     btn_raw[NUM_BUTTONS];
static int      btn_debounce[NUM_BUTTONS];
static bool     btn_prev[NUM_BUTTONS];

static adc_oneshot_unit_handle_t s_adc;
static bool     s_inited;
static int64_t  s_last_scan_us;

static float    s_vol_ema;
static float    s_filt_ema;
static float    s_dly_ema;

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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, SLIDER1_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, SLIDER2_ADC_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, POT_ADC_CHANNEL, &chan_cfg));

    ESP_LOGI(TAG, "ADC: slider1=GPIO34 ch%d | slider2=GPIO39 ch%d | pot=GPIO36 ch%d",
             SLIDER1_ADC_CHANNEL, SLIDER2_ADC_CHANNEL, POT_ADC_CHANNEL);
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

        /* Pads 0–3 */
        if (i < NUM_DRUM_PADS) {
            int idx = i;
            if (g_state.pad_mode == PAD_DRUMS) {
                note_event_post(s_pad_drum_slot[idx], 200, 1.0f, false,
                                (int8_t)(10 + idx));
            } else {
                if (shared_state_lock()) {
                    g_state.active_instrument = s_pad_instrument[idx];
                    shared_state_unlock();
                }
                ESP_LOGI(TAG, "Instrument -> %d", (int)s_pad_instrument[idx]);
            }
            continue;
        }

        switch (i) {
        case 4: /* PadTog */
            if (shared_state_lock()) {
                g_state.pad_mode = (g_state.pad_mode == PAD_INSTRUMENTS)
                                       ? PAD_DRUMS
                                       : PAD_INSTRUMENTS;
                shared_state_unlock();
            }
            ESP_LOGI(TAG, "Pad mode -> %s",
                     g_state.pad_mode == PAD_DRUMS ? "DRUMS" : "INSTRUMENTS");
            break;
        case 5: /* Record */
            if (shared_state_lock()) {
                g_state.record_pressed = true;
                shared_state_unlock();
            }
            ESP_LOGI(TAG, "Transport: REC");
            break;
        case 6: /* Clear */
            if (shared_state_lock()) {
                g_state.clear_pressed = true;
                shared_state_unlock();
            }
            ESP_LOGI(TAG, "Transport: CLEAR");
            break;
        case 7: /* Play */
            if (shared_state_lock()) {
                g_state.play_pause_pressed = true;
                shared_state_unlock();
            }
            ESP_LOGI(TAG, "Transport: PLAY/PAUSE");
            break;
        case 8: /* TrkCycle */
            if (shared_state_lock()) {
                g_state.active_track =
                    (uint8_t)((g_state.active_track + 1) % NUM_TRACKS);
                shared_state_unlock();
            }
            ESP_LOGI(TAG, "Track -> %d", (int)g_state.active_track);
            break;
        default:
            break;
        }
    }
}

static void read_analog_and_apply(void)
{
    int raw1 = 0, raw2 = 0, rawp = 0;
    if (adc_oneshot_read(s_adc, SLIDER1_ADC_CHANNEL, &raw1) != ESP_OK) {
        return;
    }
    if (adc_oneshot_read(s_adc, SLIDER2_ADC_CHANNEL, &raw2) != ESP_OK) {
        return;
    }
    if (adc_oneshot_read(s_adc, POT_ADC_CHANNEL, &rawp) != ESP_OK) {
        return;
    }

    if (raw1 < 0) {
        raw1 = 0;
    }
    if (raw2 < 0) {
        raw2 = 0;
    }
    if (rawp < 0) {
        rawp = 0;
    }

    float n1 = fminf(1.0f, (float)raw1 / 4095.0f);
    float n2 = fminf(1.0f, (float)raw2 / 4095.0f);
    float np = fminf(1.0f, (float)rawp / 4095.0f);

    const float a = 0.25f;
    s_vol_ema  += (n1 - s_vol_ema) * a;
    s_filt_ema += (n2 - s_filt_ema) * a;
    s_dly_ema  += (np - s_dly_ema) * a;

    float vol   = s_vol_ema;
    float f_hz  = MASTER_FILTER_MIN_HZ +
                  s_filt_ema * (MASTER_FILTER_MAX_HZ - MASTER_FILTER_MIN_HZ);
    float d_mix = s_dly_ema;

    if (shared_state_lock()) {
        g_state.master_volume    = vol;
        g_state.master_filter    = f_hz;
        g_state.master_delay_mix = d_mix;
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
    s_vol_ema  = 0.5f;
    s_filt_ema = 0.5f;
    s_dly_ema  = 0.0f;
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
