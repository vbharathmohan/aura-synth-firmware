#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h" // ESP-IDF v6+ new I2C driver (legacy i2c.h removed in v7)
#include "driver/i2s_std.h"
#include "led_strip.h"
#include "VL53L0X.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_attr.h" // for IRAM_ATTR
#include <atomic>     // for std::atomic (C++ atomic, works in .cpp files)

static const char *TAG = "THEREMIN_PRO";

// ── Configuration ────────────────────────────────────
#define SENSOR_COUNT 8
#define SAMPLE_RATE 44100
#define DMA_BUF_LEN 512
#define LED_STRIP_COUNT 60
#define BIN_COUNT SENSOR_COUNT
#define LEDS_PER_BIN (LED_STRIP_COUNT / BIN_COUNT)

// ── Metronome ────────────────────────────────────────
// METRONOME_LED  : index of the LED used as the metronome indicator.
//                 Set to (LED_STRIP_COUNT - 1) to claim the leftmost physical
//                 LED that was previously undriven (off) due to the 60/8 = 7
//                 integer-truncation leaving LEDs 56-59 unowned.
// METRONOME_BPM  : beats per minute — change this to set the tempo.
// METRONOME_FLASH_MS : how long the white flash stays on each beat (ms).
#define METRONOME_BPM 128 // ← change tempo here
// #define METRONOME_FLASH_MS 100 // flash on-time per beat
#define METRONOME_FLASH_MS 40 // ← "tap" attack time
#define METRONOME_TAIL_PCT 55 // ← tail length as % of beat period (match slider)
#define METRONOME_PEAK 220

// Sensor timing: budget is 20 ms, so polling faster than that is wasteful.
// We poll every 22 ms to give a small margin above the measurement budget.
#define SENSOR_POLL_MS 22

// Age threshold: a reading older than 150 ms is considered stale.
#define STALE_AGE_MS 150

// Distance thresholds (mm)
#define DIST_ACTIVE 600     // hand must be closer than this to trigger a note
#define DIST_LED_ACTIVE 700 // slightly wider zone for LED brightness
#define DIST_MIN 50         // closest expected hand distance
#define DIST_RANGE 550      // DIST_ACTIVE - DIST_MIN (used for volume scaling)

// ── Shift Register Pins ──────────────────────────────
#define SHIFT_SER GPIO_NUM_14
#define SHIFT_SRCLK GPIO_NUM_15
#define SHIFT_RCLK GPIO_NUM_32

// ── Hardware Pin Map ─────────────────────────────────
#define I2C_SDA_PIN (gpio_num_t)22
#define I2C_SCL_PIN (gpio_num_t)20
#define I2C_PWR_PIN (gpio_num_t)7

#define I2S_BCK_IO GPIO_NUM_26
#define I2S_WS_IO GPIO_NUM_25
#define I2S_DO_IO GPIO_NUM_4
#define I2S_MUTE_PIN GPIO_NUM_27
#define BUTTON_PIN GPIO_NUM_38
#define LED_STRIP_GPIO GPIO_NUM_5

const uint8_t SENSOR_ADDRS[SENSOR_COUNT] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
const uint32_t BIN_FREQS[SENSOR_COUNT] = {523, 587, 659, 698, 784, 880, 988, 1047};
const float BIN_HUES[SENSOR_COUNT] = {0, 30, 60, 120, 180, 240, 275, 310};
const int METRONOME_LED[4] = {(LED_STRIP_COUNT - 1), (LED_STRIP_COUNT - 2), (LED_STRIP_COUNT - 3), (LED_STRIP_COUNT - 4)}; // LED index 59

extern const uint8_t song_fixed_new_wav_start[] asm("_binary_song_fixed_new_wav_start");
extern const uint8_t song_fixed_new_wav_end[] asm("_binary_song_fixed_new_wav_end");

// ── Shared State ─────────────────────────────────────
typedef struct
{
    int id;
    uint16_t distance;
} sensor_msg_t;

typedef struct
{
    uint16_t distance;
    int64_t last_update_us;
} bin_state_t;

static bin_state_t bin_states[SENSOR_COUNT];
static SemaphoreHandle_t bin_mutex;
static QueueHandle_t sensor_queue;
static i2s_chan_handle_t tx_chan;
static led_strip_handle_t led_strip;
static VL53L0X *sensors[SENSOR_COUNT];

// ── Atomics for cross-core shared scalars ─────────────
// std::atomic<T> is the correct C++ mechanism for cache-coherent
// reads/writes between Core 0 (audio) and Core 1 (processor/LED).
// _Atomic is C11-only and does not compile in .cpp translation units.
static std::atomic<float> current_freq{0.0f};
static std::atomic<float> current_volume{0.0f};
static std::atomic<bool> wav_playing{false}; // toggled by button edge, not held
static std::atomic<bool> is_booting{true};
static std::atomic<bool> metronome_beat{false}; // set true momentarily by metro task
static std::atomic<uint8_t> metro_brightness{0};
// WAV playback: fractional position (16.16 fixed-point) allows variable-rate
// resampling so gestures can control pitch and volume during WAV mode.
// Only ever touched inside audio_mixer_task — no locking needed.
static uint32_t wav_pos_fp = 0;               // upper 16 bits = sample index
static uint32_t wav_file_sample_rate = 44100; // populated from WAV header at boot

// ── Precomputed sine table (replaces sinf() in hot path) ─
// 1024-entry table → phase step of 2π/1024 per sample.
// Lookup is ~10× faster than sinf() on Xtensa LX7.
#define SINE_TABLE_SIZE 1024
static float sine_table[SINE_TABLE_SIZE];

static void build_sine_table(void)
{
    for (int i = 0; i < SINE_TABLE_SIZE; i++)
        sine_table[i] = sinf(2.0f * (float)M_PI * i / SINE_TABLE_SIZE);
}

// Integer phase accumulator: avoids floating-point modulo each sample.
// phase_accum increments by phase_inc per sample.
// phase_inc = freq * SINE_TABLE_SIZE / SAMPLE_RATE  (scaled to table units)
static inline float IRAM_ATTR sine_fast(uint32_t *accum, float inc_f)
{
    *accum = (*accum + (uint32_t)inc_f) & (SINE_TABLE_SIZE - 1);
    return sine_table[*accum];
}

// ── Shift Register Logic ─────────────────────────────
static void update_xshut_register(uint8_t bitmask)
{
    for (int i = 7; i >= 0; i--)
    {
        gpio_set_level(SHIFT_SRCLK, 0);
        gpio_set_level(SHIFT_SER, (bitmask >> i) & 0x01);
        gpio_set_level(SHIFT_SRCLK, 1);
    }
    gpio_set_level(SHIFT_RCLK, 1);
    esp_rom_delay_us(1);
    gpio_set_level(SHIFT_RCLK, 0);
}

// ── HSV → RGB ─────────────────────────────────────────
static void hsv_to_rgb(float h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    float sf = s / 255.0f, vf = v / 255.0f;
    float hh = fmodf(h, 360.0f) / 60.0f;
    int i = (int)hh;
    float ff = hh - i;
    float p = vf * (1 - sf);
    float q = vf * (1 - sf * ff);
    float t = vf * (1 - sf * (1 - ff));
    switch (i)
    {
    case 0:
        *r = vf * 255;
        *g = t * 255;
        *b = p * 255;
        break;
    case 1:
        *r = q * 255;
        *g = vf * 255;
        *b = p * 255;
        break;
    case 2:
        *r = p * 255;
        *g = vf * 255;
        *b = t * 255;
        break;
    case 3:
        *r = p * 255;
        *g = q * 255;
        *b = vf * 255;
        break;
    case 4:
        *r = t * 255;
        *g = p * 255;
        *b = vf * 255;
        break;
    default:
        *r = vf * 255;
        *g = p * 255;
        *b = q * 255;
        break;
    }
}

// ── LED Boot Helpers ──────────────────────────────────
static void set_bin_color(int bin, uint8_t r, uint8_t g, uint8_t b)
{
    int start = bin * LEDS_PER_BIN;
    for (int j = 0; j < LEDS_PER_BIN; j++)
        led_strip_set_pixel(led_strip, start + j, r, g, b);
    led_strip_refresh(led_strip);
}

static void clear_strip(void)
{
    for (int i = 0; i < LED_STRIP_COUNT; i++)
        led_strip_set_pixel(led_strip, i, 0, 0, 0);
    led_strip_refresh(led_strip);
}

// ── Audio Mixer Task ──────────────────────────────────
// Placed in IRAM so cache misses don't stall the real-time audio path.
static void IRAM_ATTR audio_mixer_task(void *pvParameters)
{
    static int16_t samples[DMA_BUF_LEN * 2]; // static → no stack pressure

    const int16_t *wav_data = (const int16_t *)(song_fixed_new_wav_start + 44);
    const uint32_t wav_sample_count = (song_fixed_new_wav_end - song_fixed_new_wav_start - 44) / sizeof(int16_t);

// ── Fixed-point constants ─────────────────────────────────────────────────
// Both the sine oscillator and the WAV resampler use 16.16 fixed-point
// accumulators.  The upper 16 bits are the integer index; the lower 16 bits
// are the fractional part (sub-sample / sub-table-entry precision).
#define FRAC_BITS 16
#define FRAC_ONE (1u << FRAC_BITS)
#define FRAC_MASK (FRAC_ONE - 1u)

    uint32_t sine_accum = 0;

    // Nominal WAV advance rate: file_rate / output_rate expressed in 16.16.
    // Pre-computed once here; gesture pitch-shifting multiplies this at runtime.
    const float nominal_wav_step =
        (float)wav_file_sample_rate / (float)SAMPLE_RATE * (float)FRAC_ONE;

    // "Natural" pitch of the WAV — the sensor bin whose frequency maps to 1× speed.
    // We use the middle bin (index 3, 698 Hz) as the neutral point so gestures
    // can bend the pitch both up and down.
    const float wav_neutral_freq = (float)BIN_FREQS[SENSOR_COUNT / 2]; // 698 Hz

    while (1)
    {
        // Snapshot atomically once per buffer — never read these inside the loop
        float local_freq = current_freq.load();
        float local_vol = current_volume.load();
        bool local_wav = wav_playing.load();
        bool local_booting = is_booting.load();

        bool active = (local_freq > 0.0f || local_wav || local_booting);
        gpio_set_level(I2S_MUTE_PIN, active ? 1 : 0);

        // ── Sine oscillator increment (used only in theremin mode) ────────────
        // inc = freq * SINE_TABLE_SIZE / SAMPLE_RATE * FRAC_ONE
        uint32_t sine_inc = (local_freq > 0.0f && !local_wav)
                                ? (uint32_t)(local_freq * (float)SINE_TABLE_SIZE / (float)SAMPLE_RATE * (float)FRAC_ONE)
                                : 0u;

        // ── WAV resample step (used only in WAV mode) ─────────────────────────
        // When no hand is detected, play at nominal speed (1×).
        // When a hand is detected, scale speed by (detected_freq / neutral_freq).
        // This bends the pitch up/down relative to the WAV's original pitch.
        // Volume comes from current_volume exactly as in sine mode.
        float wav_pitch_ratio = 1.0f;
        float wav_vol_scale = 1.0f;
        if (local_wav)
        {
            if (local_freq > 0.0f)
            {
                wav_pitch_ratio = local_freq / wav_neutral_freq;
                wav_vol_scale = local_vol;
            }
            // else: no hand → play at normal speed, full volume
        }
        uint32_t wav_step = (uint32_t)(nominal_wav_step * wav_pitch_ratio);

        float sine_vol = local_vol * 12000.0f;
        float wav_vol = wav_vol_scale * 12000.0f;

        for (int i = 0; i < DMA_BUF_LEN; i++)
        {
            float mixed = 0.0f;

            // ── Theremin sine tone ────────────────────────────────────────────
            if (sine_inc)
            {
                sine_accum += sine_inc;
                uint32_t idx = (sine_accum >> FRAC_BITS) & (SINE_TABLE_SIZE - 1u);
                mixed += sine_table[idx] * sine_vol;
            }

            // ── WAV playback with linear interpolation ────────────────────────
            // Linear interpolation between adjacent samples removes the aliasing
            // artifacts that appear when pitch-shifting by integer steps.
            if (local_wav)
            {
                uint32_t idx0 = (wav_pos_fp >> FRAC_BITS) % wav_sample_count;
                uint32_t idx1 = (idx0 + 1u) % wav_sample_count;
                float frac = (float)(wav_pos_fp & FRAC_MASK) / (float)FRAC_ONE;
                float s = (float)wav_data[idx0] * (1.0f - frac) + (float)wav_data[idx1] * frac;
                mixed += s * wav_vol;
                wav_pos_fp += wav_step;
                // Keep accumulator in range (wrap at sample_count in 16.16)
                if ((wav_pos_fp >> FRAC_BITS) >= wav_sample_count)
                    wav_pos_fp -= (wav_sample_count << FRAC_BITS);
            }

            // ── Clamp and emit ────────────────────────────────────────────────
            int32_t out = (int32_t)mixed;
            if (out > 32767)
                out = 32767;
            if (out < -32768)
                out = -32768;

            samples[i * 2] = (int16_t)out;
            samples[i * 2 + 1] = (int16_t)out;
        }

        size_t written;
        i2s_channel_write(tx_chan, samples, sizeof(samples), &written, portMAX_DELAY);
    }
#undef FRAC_BITS
#undef FRAC_ONE
#undef FRAC_MASK
}

// ── Processor Task ────────────────────────────────────
static void processor_task(void *pvParameters)
{
    sensor_msg_t incoming;

    // ── Button debounce state ─────────────────────────────────────────────────
    // We detect a falling edge (HIGH → LOW) and toggle wav_playing on each one.
    // A 50 ms debounce window prevents contact-bounce from firing twice.
    bool btn_prev = true;              // last stable button state (HIGH = not pressed)
    int64_t btn_last_edge = 0;         // µs timestamp of last confirmed edge
    const int64_t DEBOUNCE_US = 50000; // 50 ms

    while (1)
    {
        // Drain all pending sensor messages before recalculating.
        while (xQueueReceive(sensor_queue, &incoming, pdMS_TO_TICKS(SENSOR_POLL_MS)))
        {
            xSemaphoreTake(bin_mutex, portMAX_DELAY);
            bin_states[incoming.id].distance = incoming.distance;
            bin_states[incoming.id].last_update_us = esp_timer_get_time();
            xSemaphoreGive(bin_mutex);
        }

        if (is_booting.load())
            continue;

        // ── Debounced toggle on button press ──────────────────────────────────
        bool btn_now = (gpio_get_level(BUTTON_PIN) != 0); // true = not pressed
        int64_t now_us = esp_timer_get_time();
        if (!btn_now && btn_prev && (now_us - btn_last_edge) > DEBOUNCE_US)
        {
            // Falling edge detected — toggle WAV mode
            wav_playing.store(!wav_playing.load());
            btn_last_edge = now_us;
            ESP_LOGI("PROC", "WAV mode %s", wav_playing.load() ? "ON" : "OFF");
        }
        btn_prev = btn_now;

        // ── Find closest active sensor bin ───────────────────────────────────
        xSemaphoreTake(bin_mutex, portMAX_DELAY);
        int closest = -1;
        uint16_t min_d = DIST_ACTIVE;
        int64_t now = esp_timer_get_time();

        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            int64_t age_ms = (now - bin_states[i].last_update_us) / 1000;
            if (bin_states[i].distance < DIST_ACTIVE && age_ms < STALE_AGE_MS)
            {
                if (bin_states[i].distance < min_d)
                {
                    min_d = bin_states[i].distance;
                    closest = i;
                }
            }
        }
        xSemaphoreGive(bin_mutex);

        // ── Publish freq + volume regardless of mode ──────────────────────────
        // The audio task reads these in both sine mode and WAV mode, so we
        // always keep them up to date.  When no hand is present, they are 0
        // and the WAV plays at nominal speed/volume (handled in audio task).
        if (closest != -1)
        {
            float vol = 1.0f - ((float)(min_d - DIST_MIN) / (float)DIST_RANGE);
            if (vol < 0.0f)
                vol = 0.0f;
            if (vol > 1.0f)
                vol = 1.0f;
            current_freq.store((float)BIN_FREQS[closest]);
            current_volume.store(vol);
        }
        else
        {
            current_freq.store(0.0f);
            current_volume.store(0.0f);
        }
    }
}

// ── LED Task ──────────────────────────────────────────
static void led_task(void *pvParameters)
{
    float smooth_v[SENSOR_COUNT] = {0};

    // Local copy of bin state — avoids holding the mutex during refresh.
    uint16_t local_dist[SENSOR_COUNT];
    int64_t local_age_ms[SENSOR_COUNT];

    while (1)
    {
        if (is_booting.load())
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // ── 1. Critical section: snapshot bin state only ──────────
        {
            int64_t now = esp_timer_get_time();
            xSemaphoreTake(bin_mutex, portMAX_DELAY);
            for (int b = 0; b < SENSOR_COUNT; b++)
            {
                local_dist[b] = bin_states[b].distance;
                local_age_ms[b] = (now - bin_states[b].last_update_us) / 1000;
            }
            xSemaphoreGive(bin_mutex);
            // Mutex released — refresh can now take as long as it needs.
        }

        // ── 2. Compute colors and write pixels (no mutex held) ────
        for (int b = 0; b < SENSOR_COUNT; b++)
        {
            float target = (local_dist[b] < DIST_LED_ACTIVE && local_age_ms[b] < STALE_AGE_MS)
                               ? 1.0f
                               : 0.0f;
            smooth_v[b] += (target - smooth_v[b]) * 0.2f;

            uint8_t r, g, bl;
            uint8_t val = (uint8_t)(smooth_v[b] * 200);
            hsv_to_rgb(BIN_HUES[b], 255, val, &r, &g, &bl);

            int start = b * LEDS_PER_BIN;
            for (int j = 0; j < LEDS_PER_BIN; j++)
                led_strip_set_pixel(led_strip, start + j, r, g, bl);
        }

        // ── 3. Overlay metronome LED (outside mutex, after bin writes) ──
        // METRONOME_LED is index 59 — never written by the bin loop (bins cover
        // 0-55 only due to 60/8 integer truncation), so no write conflict.
        // if (metronome_beat.load())
        // {
        for (int i = 0; i < sizeof(METRONOME_LED) / sizeof(METRONOME_LED[0]); i++)
        {
            uint8_t mv = metro_brightness.load();
            led_strip_set_pixel(led_strip, METRONOME_LED[i], mv, mv, mv);
        }
        // }
        // else
        // {
        //     for (int i = 0; i < sizeof(METRONOME_LED) / sizeof(METRONOME_LED[0]); i++)
        //         led_strip_set_pixel(led_strip, METRONOME_LED[i], 0, 0, 0); // off between beats
        // }

        // ── 4. Refresh strip outside the mutex ───────────────────
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ── Metronome Task ───────────────────────────────────
// Fires a beat flag at METRONOME_BPM.  The LED task reads the flag and
// renders the flash; this task just manages timing and the atomic flag.
// Keeping it separate means the beat is never delayed by I2C or LED refresh.
static void metronome_task(void *pvParameters)
{
    const TickType_t beat_ticks = pdMS_TO_TICKS(60000 / METRONOME_BPM);
    const TickType_t flash_ticks = pdMS_TO_TICKS(METRONOME_FLASH_MS);

    const int beat_ms = 60000 / METRONOME_BPM;
    const int tail_ms = beat_ms * METRONOME_TAIL_PCT / 100;

    while (1)
    {
        if (is_booting.load())
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // ── Attack: ramp up over METRONOME_FLASH_MS ───────────────
        const int STEP_MS = 10;
        const int atk_steps = METRONOME_FLASH_MS / STEP_MS;
        for (int s = 0; s <= atk_steps; s++)
        {
            uint8_t v = (uint8_t)(METRONOME_PEAK * s / atk_steps);
            metro_brightness.store(v);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }

        // ── Decay: exponential tail until tail_ms elapsed ─────────
        int elapsed = METRONOME_FLASH_MS;
        while (elapsed < tail_ms)
        {
            float x = (float)(elapsed - METRONOME_FLASH_MS) / (float)(tail_ms - METRONOME_FLASH_MS);
            float norm = powf(1.0f - x, 2.4f); // power curve matches preview
            metro_brightness.store((uint8_t)(METRONOME_PEAK * norm));
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
            elapsed += STEP_MS;
        }

        metro_brightness.store(0);

        // ── Wait out the rest of the beat ─────────────────────────
        int wait = beat_ms - tail_ms;
        if (wait > 0)
            vTaskDelay(pdMS_TO_TICKS(wait));
    }
}

// ── Sensor Manager Task ───────────────────────────────
// Polls at SENSOR_POLL_MS — there is no benefit polling faster than the
// sensor's own timing budget (20 ms), so 22 ms gives a clean margin.
static void sensor_manager_task(void *pvParameters)
{
    sensor_msg_t msg;
    VL53L0X_RangingMeasurementData_t data;

    while (1)
    {
        if (is_booting.load())
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            if (sensors[i] == NULL)
                continue;

            VL53L0X_Dev_t *dev = sensors[i]->get_dev();

            // Only read if data is actually ready (avoids stale/duplicate reads)
            uint8_t ready = 0;
            VL53L0X_GetMeasurementDataReady(dev, &ready);
            if (!ready)
                continue;

            if (VL53L0X_GetRangingMeasurementData(dev, &data) == VL53L0X_ERROR_NONE && data.RangeStatus == 0)
            {
                msg.id = i;
                msg.distance = data.RangeMilliMeter;
                xQueueSend(sensor_queue, &msg, 0);
            }

            VL53L0X_ClearInterruptMask(dev, 0);
        }

        // Sleep until the next measurement cycle is due.
        // Previously 1 ms → CPU was spinning 20× more than needed.
        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
    }
}

// ── Initialization ────────────────────────────────────
static void init_hardware(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << SHIFT_SER) | (1ULL << SHIFT_SRCLK) | (1ULL << SHIFT_RCLK);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    update_xshut_register(0x00);

    led_strip_config_t sc = {};
    sc.strip_gpio_num = LED_STRIP_GPIO;
    sc.max_leds = LED_STRIP_COUNT;
    sc.led_model = LED_MODEL_WS2812;
    sc.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

    led_strip_rmt_config_t rc = {};
    rc.clk_src = RMT_CLK_SRC_DEFAULT;
    rc.resolution_hz = 10 * 1000 * 1000;
    rc.mem_block_symbols = 64;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc, &rc, &led_strip));

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}}};

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));

    gpio_set_direction(I2S_MUTE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(I2C_PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(I2C_PWR_PIN, 1);
}

// ── Entry Point ───────────────────────────────────────
extern "C" void app_main(void)
{
    is_booting.store(true);

    build_sine_table(); // Must happen before audio task starts

    // ── Read WAV sample rate from header ─────────────────────────────────────
    // Bytes 24-27 of a standard WAV file contain the sample rate as a
    // little-endian uint32.  Reading it here prevents the "plays too fast"
    // bug that occurs when the file rate differs from SAMPLE_RATE.
    if (song_fixed_new_wav_end - song_fixed_new_wav_start >= 28)
    {
        const uint8_t *h = song_fixed_new_wav_start;
        wav_file_sample_rate = (uint32_t)h[24] | ((uint32_t)h[25] << 8) | ((uint32_t)h[26] << 16) | ((uint32_t)h[27] << 24);
        ESP_LOGI(TAG, "WAV sample rate: %lu Hz", (unsigned long)wav_file_sample_rate);
    }

    init_hardware();

    sensor_queue = xQueueCreate(30, sizeof(sensor_msg_t));
    bin_mutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(audio_mixer_task, "audio", 4096, NULL, 15, NULL, 0);
    xTaskCreatePinnedToCore(processor_task, "proc", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(led_task, "leds", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(sensor_manager_task, "snr_mgr", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(metronome_task, "metro", 2048, NULL, 6, NULL, 1);

    vTaskDelay(pdMS_TO_TICKS(100)); // Let audio task pre-fill DMA buffers

    VL53L0X temp_bus(I2C_NUM_0);
    temp_bus.i2cMasterInit(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

    // ── PHASE 1: White Flash POST ─────────────────────
    ESP_LOGI(TAG, "LED POST: White Flash");
    for (int i = 0; i < LED_STRIP_COUNT; i++)
        led_strip_set_pixel(led_strip, i, 50, 50, 50);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(800));
    clear_strip();
    vTaskDelay(pdMS_TO_TICKS(200));

    // ── PHASE 2: Sensor Activation ────────────────────
    uint8_t mask = 0;
    bool all_ok = true;
    current_volume.store(1.0f);

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        mask |= (1 << i);
        update_xshut_register(mask);
        vTaskDelay(pdMS_TO_TICKS(60));

        sensors[i] = new VL53L0X(I2C_NUM_0, GPIO_NUM_MAX, GPIO_NUM_MAX);
        if (sensors[i]->init())
        {
            sensors[i]->setDeviceAddress(SENSOR_ADDRS[i]);
            vTaskDelay(pdMS_TO_TICKS(15));
            sensors[i]->setTimingBudget(20000);

            VL53L0X_SetDeviceMode(sensors[i]->get_dev(), VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
            VL53L0X_StartMeasurement(sensors[i]->get_dev());

            set_bin_color(i, 0, 100, 0);
            current_freq.store((float)BIN_FREQS[i]);
            vTaskDelay(pdMS_TO_TICKS(150));
            current_freq.store(0.0f);
        }
        else
        {
            ESP_LOGE(TAG, "Sensor %d Failed", i);
            sensors[i] = NULL;
            all_ok = false;
            set_bin_color(i, 100, 0, 0);
            current_freq.store(150.0f);
            vTaskDelay(pdMS_TO_TICKS(300));
            current_freq.store(0.0f);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    current_volume.store(0.0f);

    // ── PHASE 3: Sweep → Go Live ──────────────────────
    ESP_LOGI(TAG, "Boot Complete: Swipe Sequence");
    current_volume.store(1.0f);
    const float start_freq = 110.0f;
    const float end_freq = 880.0f;

    for (int i = 0; i < LED_STRIP_COUNT; i++)
    {
        clear_strip();
        led_strip_set_pixel(led_strip, (LED_STRIP_COUNT - 1) - i, 100, 100, 100);
        led_strip_refresh(led_strip);
        current_freq.store(start_freq + (end_freq - start_freq) * ((float)i / LED_STRIP_COUNT));
        esp_rom_delay_us(10000);
    }

    current_freq.store(0.0f);
    current_volume.store(0.0f);
    clear_strip();

    is_booting.store(false);
    ESP_LOGI(TAG, "Theremin Active");
}