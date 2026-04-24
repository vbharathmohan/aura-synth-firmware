/**
 * main.c — Aura-Synth entry point.
 *
 * Currently: Batch 1 test harness.
 * Tests shared state, block pool, and I2S output with a sine tone.
 *
 * Once Batch 2 and 3 are added, this becomes the real entry point
 * that launches the audio task on Core 1 and sensor task on Core 0.
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "shared_state.h"
#include "audio_block.h"
#include "i2s_output.h"

static const char *TAG = "main";

/* ------------------------------------------------------------------ */
/* Batch 1 test: sine wave generator using block pool + I2S            */
/* ------------------------------------------------------------------ */

/**
 * Generates a sine wave directly through the block pipeline.
 * This proves: pool alloc → fill → convert → I2S write → free.
 * You should hear a 440 Hz tone on the DAC when this runs.
 */
static void test_audio_task(void *param)
{
    ESP_LOGI(TAG, "Test audio task started on core %d", xPortGetCoreID());

    /* DMA-capable output buffer for I2S (interleaved int16 L/R) */
    int16_t *i2s_buf = (int16_t *)heap_caps_malloc(
        BLOCK_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_DMA);

    if (i2s_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer");
        vTaskDelete(NULL);
        return;
    }

    float phase = 0.0f;
    const float freq = 440.0f;  /* A4 */
    const float amp  = 0.3f;    /* gentle volume */

    int block_count = 0;

    while (1) {
        /* 1. Allocate a block from the pool */
        audio_block_t *blk = audio_alloc();
        if (blk == NULL) {
            ESP_LOGW(TAG, "Pool exhausted, skipping frame");
            vTaskDelay(1);
            continue;
        }

        /* 2. Fill with sine wave */
        for (int i = 0; i < BLOCK_SAMPLES; i++) {
            int32_t sample = (int32_t)(sinf(phase) * amp * 32767.0f);
            blk->L[i] = sample;
            blk->R[i] = sample;

            phase += (2.0f * (float)M_PI * freq) / (float)SAMPLE_RATE;
            if (phase > 2.0f * (float)M_PI)
                phase -= 2.0f * (float)M_PI;
        }

        /* 3. Convert int32 block → interleaved int16 for I2S */
        audio_block_to_i2s(blk, i2s_buf, 1);

        /* 4. Return block to pool */
        audio_free(blk);

        /* 5. Write to I2S (blocks until DMA accepts) */
        i2s_output_write(i2s_buf, BLOCK_SAMPLES * 2 * sizeof(int16_t));

        /* Periodic status */
        block_count++;
        if (block_count % 200 == 0) {
            ESP_LOGI(TAG, "Blocks processed: %d, pool available: %d/%d",
                     block_count, audio_pool_available(), POOL_SIZE);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Placeholder: Core 0 sensor task (teammate fills this in)            */
/* ------------------------------------------------------------------ */

static void test_sensor_task(void *param)
{
    ESP_LOGI(TAG, "Sensor stub on core %d", xPortGetCoreID());

    int tick = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (shared_state_lock()) {
            /* Simulate gesture: sweep pitch slowly */
            g_state.tracks[0].pitch = 48 + (tick % 24);
            g_state.tracks[0].volume = 0.7f;
            shared_state_unlock();
        }

        if (tick % 50 == 0) {
            shared_state_t snap = {0};
            shared_state_snapshot(&snap);
            ESP_LOGI(TAG, "State: track=%d pitch=%d vol=%.2f mode=%d",
                     snap.active_track,
                     snap.tracks[0].pitch,
                     snap.tracks[0].volume,
                     snap.mode);
        }

        tick++;
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Aura-Synth starting (Batch 1 test) ===");
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());

    /* Check PSRAM */
    size_t psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    if (psram > 0) {
        ESP_LOGI(TAG, "PSRAM available: %lu KB", (unsigned long)(psram / 1024));
    } else {
        ESP_LOGW(TAG, "No PSRAM detected — delay/loop features will be limited");
    }

    /* 1. Shared state */
    shared_state_init();
    ESP_LOGI(TAG, "Shared state initialized (%d bytes)",
             (int)sizeof(shared_state_t));

    /* 2. Block pool */
    if (!audio_pool_init()) {
        ESP_LOGE(TAG, "Block pool init failed — aborting");
        return;
    }
    ESP_LOGI(TAG, "Free heap after pool: %lu bytes",
             (unsigned long)esp_get_free_heap_size());

    /* 3. I2S output */
    if (!i2s_output_init()) {
        ESP_LOGE(TAG, "I2S init failed — aborting");
        return;
    }

    /* 4. Launch test audio task on Core 1 */
    xTaskCreatePinnedToCore(
        test_audio_task, "audio_test",
        4096, NULL,
        configMAX_PRIORITIES - 1, NULL,
        1  /* Core 1 */
    );
    ESP_LOGI(TAG, "Audio test task launched on Core 1");

    /* 5. Launch sensor stub on Core 0 */
    xTaskCreatePinnedToCore(
        test_sensor_task, "sensor_stub",
        4096, NULL,
        5, NULL,
        0  /* Core 0 */
    );
    ESP_LOGI(TAG, "Sensor stub launched on Core 0");

    ESP_LOGI(TAG, "=== Batch 1 test running ===");
    ESP_LOGI(TAG, "Expected: 440 Hz sine tone on DAC, state logs every 5s");
}
