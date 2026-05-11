/**
 * led_task.cpp — WS2812 strip driver + per-bin proximity feedback +
 *                metronome overlay.
 *
 * Adapted from Lennarts_code_base/Lens_sensors/main/main.cpp's
 * led_task() and metronome_task(). Uses the same numbers Lennart
 * tuned (DIST_LED_ACTIVE = 700 mm, METRONOME_PEAK = 220, attack =
 * 40 ms, exponential 2.4-power tail at 55% of beat) so the strip
 * looks identical to the prototype.
 *
 * Two FreeRTOS tasks live in this file:
 *
 *   render_task    : 50 Hz redraw. Per bin: read latest sensor
 *                    distance, EMA-smooth a "presence" value, map to
 *                    a per-bin hue, write LEDS_PER_BIN pixels. Then
 *                    overlay the metronome brightness onto the spare
 *                    tail of the strip and refresh.
 *
 *   metronome_task : Pure timing loop that updates one atomic
 *                    `metro_brightness` byte. Decoupled from rendering
 *                    so I2C / sensor latency can never delay a beat.
 *
 * Reads sensor data through sensor_task_last_distance() — no shared
 * mutex with the audio path, so the LED stack is fully optional and
 * safe to disable.
 */

#include "led_task.h"
#include "sensor_task.h"
#include "shared_state.h"

#include <math.h>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "led_strip.h"

static const char *TAG = "led_task";

/* ------------------------------------------------------------------ */
/* Layout                                                              */
/* ------------------------------------------------------------------ */

#define LEDS_PER_BIN (LED_STRIP_COUNT / NUM_TOF_SENSORS)
#define BIN_REGION_LEDS (LEDS_PER_BIN * NUM_TOF_SENSORS)
/* Spare tail (LED indices BIN_REGION_LEDS .. LED_STRIP_COUNT-1) is
 * dedicated to the metronome overlay. For the default 60-LED strip
 * this is the 4 LEDs at indices 56..59.
 *
 * ToF bins are mapped reversed along the strip (sensor 0 at the far
 * end of the bin region) so physical wiring matches the enclosure;
 * metronome indices are unchanged. */

/* Per-bin hue spread around the wheel (Lennart's palette). */
static const float BIN_HUES[NUM_TOF_SENSORS] = {
    0.0f, 30.0f, 60.0f, 120.0f, 180.0f, 240.0f, 275.0f, 310.0f};

/* Distance at which a bin's LEDs reach full bin brightness, in mm. */
#define DIST_LED_ACTIVE_MM 700
#define BIN_PEAK_VALUE 200 /* 0..255 */
#define SENSOR_STALE_MS 150

/* ------------------------------------------------------------------ */
/* Metronome shape                                                     */
/* ------------------------------------------------------------------ */

#define METRONOME_FLASH_MS 40 /* attack time */
#define METRONOME_TAIL_PCT 55 /* tail length as % of beat */
#define METRONOME_PEAK 220
#define METRONOME_DECAY_POW 2.4f /* tail shape exponent */

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

static led_strip_handle_t s_strip = nullptr;

/* Render task -> hardware: smoothed proximity 0..1 per bin. */
static float s_smooth_v[NUM_TOF_SENSORS] = {0};

/* Cross-task atomics. */
static std::atomic<uint8_t> s_metro_brightness{0};
static std::atomic<int> s_metro_bpm{METRONOME_BPM};

static bool s_initialised = false;
static TaskHandle_t s_render_task = nullptr;
static TaskHandle_t s_metro_task = nullptr;

/* ------------------------------------------------------------------ */
/* HSV -> RGB                                                          */
/* ------------------------------------------------------------------ */

static void hsv_to_rgb(float h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    float sf = s / 255.0f, vf = v / 255.0f;
    float hh = fmodf(h, 360.0f) / 60.0f;
    int i = (int)hh;
    float ff = hh - i;
    float p = vf * (1.0f - sf);
    float q = vf * (1.0f - sf * ff);
    float t = vf * (1.0f - sf * (1.0f - ff));
    switch (i)
    {
    case 0:
        *r = (uint8_t)(vf * 255);
        *g = (uint8_t)(t * 255);
        *b = (uint8_t)(p * 255);
        break;
    case 1:
        *r = (uint8_t)(q * 255);
        *g = (uint8_t)(vf * 255);
        *b = (uint8_t)(p * 255);
        break;
    case 2:
        *r = (uint8_t)(p * 255);
        *g = (uint8_t)(vf * 255);
        *b = (uint8_t)(t * 255);
        break;
    case 3:
        *r = (uint8_t)(p * 255);
        *g = (uint8_t)(q * 255);
        *b = (uint8_t)(vf * 255);
        break;
    case 4:
        *r = (uint8_t)(t * 255);
        *g = (uint8_t)(p * 255);
        *b = (uint8_t)(vf * 255);
        break;
    default:
        *r = (uint8_t)(vf * 255);
        *g = (uint8_t)(p * 255);
        *b = (uint8_t)(q * 255);
        break;
    }
}

static inline bool led_ready(void)
{
    return s_initialised && (s_strip != nullptr);
}

/** First LED index for ToF bin `bin` (strip reversed vs sensor order). */
static inline int strip_bin_start(int bin)
{
    return (NUM_TOF_SENSORS - 1 - bin) * LEDS_PER_BIN;
}

static void set_bin_rgb(int bin, uint8_t r, uint8_t g, uint8_t b)
{
    if (!led_ready())
        return;
    if (bin < 0 || bin >= NUM_TOF_SENSORS)
        return;
    int start = strip_bin_start(bin);
    for (int j = 0; j < LEDS_PER_BIN; j++)
    {
        led_strip_set_pixel(s_strip, start + j, r, g, b);
    }
}

/* ------------------------------------------------------------------ */
/* Render task                                                         */
/* ------------------------------------------------------------------ */

static void led_render_task(void *)
{
    ESP_LOGI(TAG, "Render task on core %d, %d LEDs (%d bin LEDs + %d spare)",
             xPortGetCoreID(), LED_STRIP_COUNT,
             BIN_REGION_LEDS, LED_STRIP_COUNT - BIN_REGION_LEDS);

    while (true)
    {
        /* --- 1. Per-bin presence (closer hand = brighter) --- */
        for (int b = 0; b < NUM_TOF_SENSORS; b++)
        {
            uint16_t d = sensor_task_last_distance(b);
            int32_t age_ms = sensor_task_last_age_ms(b);
            float target = (d > 0 && d < DIST_LED_ACTIVE_MM && age_ms <= SENSOR_STALE_MS)
                               ? 1.0f
                               : 0.0f;
            /* EMA smoothing — same coefficient Lennart's prototype uses. */
            s_smooth_v[b] += (target - s_smooth_v[b]) * 0.2f;

            uint8_t r, g, bl;
            uint8_t val = (uint8_t)(s_smooth_v[b] * BIN_PEAK_VALUE);
            hsv_to_rgb(BIN_HUES[b], 255, val, &r, &g, &bl);

            int start = strip_bin_start(b);
            for (int j = 0; j < LEDS_PER_BIN; j++)
            {
                led_strip_set_pixel(s_strip, start + j, r, g, bl);
            }
        }

        /* --- 2. Metronome overlay on the spare tail (white flash) --- */
        uint8_t m = s_metro_brightness.load();
        for (int i = BIN_REGION_LEDS; i < LED_STRIP_COUNT; i++)
        {
            led_strip_set_pixel(s_strip, i, m, m, m);
        }

        /* --- 3. Push to the strip --- */
        led_strip_refresh(s_strip);

        vTaskDelay(pdMS_TO_TICKS(20)); /* ~50 Hz redraw */
    }
}

/* ------------------------------------------------------------------ */
/* Metronome task                                                      */
/* ------------------------------------------------------------------ */

static void metronome_task(void *)
{
    ESP_LOGI(TAG, "Metronome task on core %d, %d BPM",
             xPortGetCoreID(), s_metro_bpm.load());

    const int STEP_MS = 10; /* update granularity */

    while (true)
    {
        int bpm = s_metro_bpm.load();
        if (bpm <= 0)
        {
            s_metro_brightness.store(0);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        const int beat_ms = 60000 / bpm;
        const int tail_ms = beat_ms * METRONOME_TAIL_PCT / 100;

        /* Attack: linear ramp 0 -> peak over METRONOME_FLASH_MS. */
        const int atk_steps = METRONOME_FLASH_MS / STEP_MS;
        for (int s = 0; s <= atk_steps; s++)
        {
            uint8_t v = (uint8_t)(METRONOME_PEAK * s / atk_steps);
            s_metro_brightness.store(v);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }

        /* Decay: power-law tail until tail_ms elapsed. */
        int elapsed = METRONOME_FLASH_MS;
        while (elapsed < tail_ms)
        {
            float x = (float)(elapsed - METRONOME_FLASH_MS) /
                      (float)(tail_ms - METRONOME_FLASH_MS);
            float norm = powf(1.0f - x, METRONOME_DECAY_POW);
            s_metro_brightness.store((uint8_t)(METRONOME_PEAK * norm));
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
            elapsed += STEP_MS;
        }

        s_metro_brightness.store(0);

        /* Wait out the rest of the beat. */
        int wait = beat_ms - tail_ms;
        if (wait > 0)
            vTaskDelay(pdMS_TO_TICKS(wait));
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

bool led_task_init(void)
{
    if (s_initialised)
        return true;

    led_strip_config_t sc = {};
    sc.strip_gpio_num = LED_STRIP_GPIO;
    sc.max_leds = LED_STRIP_COUNT;
    sc.led_pixel_format = LED_PIXEL_FORMAT_GRB;
    sc.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t rc = {};
    rc.clk_src = RMT_CLK_SRC_DEFAULT;
    rc.resolution_hz = 10 * 1000 * 1000; /* 10 MHz */
    rc.mem_block_symbols = 64;

    esp_err_t err = led_strip_new_rmt_device(&sc, &rc, &s_strip);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    /* Blank the strip on boot. */
    for (int i = 0; i < LED_STRIP_COUNT; i++)
    {
        led_strip_set_pixel(s_strip, i, 0, 0, 0);
    }
    led_strip_refresh(s_strip);

    s_initialised = true;
    ESP_LOGI(TAG, "Init OK: GPIO %d, %d LEDs, %d per bin",
             LED_STRIP_GPIO, LED_STRIP_COUNT, LEDS_PER_BIN);
    return true;
}

void led_task_start(void)
{
    if (!s_initialised)
    {
        ESP_LOGE(TAG, "led_task_start() before successful _init()");
        return;
    }
    if (s_render_task == nullptr)
    {
        xTaskCreatePinnedToCore(led_render_task,
                                "led", 4096, nullptr,
                                4, &s_render_task, 1);
    }
    if (s_metro_task == nullptr)
    {
        xTaskCreatePinnedToCore(metronome_task,
                                "metro", 2048, nullptr,
                                5, &s_metro_task, 1);
    }
}

void led_task_set_bpm(int bpm)
{
    if (bpm < 0)
        bpm = 0;
    if (bpm > 300)
        bpm = 300;
    s_metro_bpm.store(bpm);
}

void led_task_boot_clear(void)
{
    if (!led_ready())
        return;
    for (int i = 0; i < LED_STRIP_COUNT; i++)
    {
        led_strip_set_pixel(s_strip, i, 0, 0, 0);
    }
    led_strip_refresh(s_strip);
}

void led_task_boot_set_sensor_ready(int sensor_idx, bool ready)
{
    if (!led_ready())
        return;
    if (ready)
    {
        uint8_t r, g, b;
        hsv_to_rgb(BIN_HUES[sensor_idx], 255, BIN_PEAK_VALUE, &r, &g, &b);
        set_bin_rgb(sensor_idx, r, g, b);
    }
    else
    {
        set_bin_rgb(sensor_idx, 0, 0, 0);
    }
    led_strip_refresh(s_strip);
}

void led_task_boot_swoop_animation(int duration_ms)
{
    if (!led_ready())
        return;

    int steps = 20; // Number of animation frames
    int frame_time = duration_ms / steps;

    // Sweep from right (sensor 7) to left (sensor 0)
    for (int s = 0; s < steps; s++)
    {
        led_task_boot_clear();

        // Calculate lead pixel position (reversed mapping)
        float progress = (float)s / steps;
        int lead_pixel = (int)(progress * LED_STRIP_COUNT);

        // Draw a small "tail" of white pixels
        for (int t = 0; t < 5; t++)
        {
            int p = lead_pixel - t;
            if (p >= 0 && p < LED_STRIP_COUNT)
            {
                uint8_t br = 255 / (t + 1); // Fading tail
                led_strip_set_pixel(s_strip, p, br, br, br);
            }
        }

        led_strip_refresh(s_strip);
        vTaskDelay(pdMS_TO_TICKS(frame_time));
    }
    led_task_boot_clear();
}

void led_task_boot_flash_white(uint8_t level, int duration_ms)
{
    if (!led_ready())
        return;
    for (int i = 0; i < LED_STRIP_COUNT; i++)
    {
        led_strip_set_pixel(s_strip, i, level, level, level);
    }
    led_strip_refresh(s_strip);
    if (duration_ms > 0)
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
}
