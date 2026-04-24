/**
 * main.c — Aura-Synth entry point (production).
 *
 * Initializes all subsystems and launches two tasks:
 *   Core 0: sensor/control task (teammate's code — stub included)
 *   Core 1: audio_task_run (mixer + loop recorder + I2S)
 *
 * To switch from the test stub to real hardware:
 *   Replace sensor_stub_task with your teammate's control_task.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "shared_state.h"
#include "audio_block.h"
#include "i2s_output.h"
#include "sampler.h"
#include "mixer.h"
#include "loop_recorder.h"
#include "audio_task.h"
#include "effects.h"

static const char *TAG = "main";

/* ------------------------------------------------------------------ */
/* Embedded WAV samples — uncomment when you add files to samples/     */
/* ------------------------------------------------------------------ */


 extern const uint8_t kick_wav_start[]  asm("_binary_kick_wav_start");
 extern const uint8_t kick_wav_end[]    asm("_binary_kick_wav_end");
 extern const uint8_t snare_wav_start[] asm("_binary_snare_wav_start");
 extern const uint8_t snare_wav_end[]   asm("_binary_snare_wav_end");
 extern const uint8_t hihat_wav_start[] asm("_binary_hihat_wav_start");
 extern const uint8_t hihat_wav_end[]   asm("_binary_hihat_wav_end");
 

/* ------------------------------------------------------------------ */
/* Master delay effect (PSRAM-backed, one instance on master bus)       */
/* ------------------------------------------------------------------ */

static delay_t s_master_delay;

/* ------------------------------------------------------------------ */
/* Sensor stub (Core 0) — replace with teammate's real code            */
/* ------------------------------------------------------------------ */

static void sensor_stub_task(void *param)
{
    ESP_LOGI(TAG, "Sensor stub on core %d", xPortGetCoreID());

    int tick = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));  /* 100 Hz control rate */

        if (shared_state_lock()) {

            if (tick < 1000) {
                /* 0-10s: synth mode — sweep pitch with modulation */
                g_state.mode = MODE_SYNTH;
                g_state.active_track = 0;
                g_state.tracks[0].pitch = 48 + (tick / 40) % 24;
                g_state.tracks[0].volume = 0.6f;
                g_state.tracks[0].waveform_mix = 0.3f;
                g_state.tracks[0].lfo_rate = 5.0f;
                g_state.tracks[0].filter_cutoff = 800.0f + (tick % 500) * 2.0f;
                g_state.tracks[0].detune = 1.5f;
                g_state.master_volume = 0.7f;

            } else if (tick < 2000) {
                /* 10-20s: drum mode — trigger every 250ms */
                g_state.mode = MODE_DRUMS;
                if (tick % 25 == 0) {
                    g_state.drum.slot = ((tick - 1000) / 25) % NUM_DRUM_SLOTS;
                    g_state.drum.velocity = 180;
                    g_state.drum.trigger = true;
                }
                g_state.master_volume = 0.8f;

            } else if (tick == 2000) {
                /* 20s: test record button */
                g_state.mode = MODE_SYNTH;
                g_state.record_pressed = true;
                ESP_LOGI(TAG, "Stub: pressing RECORD");

            } else if (tick < 2500) {
                /* 20-25s: recording — vary pitch */
                g_state.tracks[0].pitch = 60 + (tick - 2000) / 20;
                g_state.tracks[0].volume = 0.5f;

            } else if (tick == 2500) {
                /* 25s: stop recording */
                g_state.record_pressed = true;
                ESP_LOGI(TAG, "Stub: pressing RECORD (stop)");

            } else {
                /* 25s+: loop plays back, synth continues */
                g_state.tracks[0].pitch = 72;
                g_state.tracks[0].volume = 0.4f;
            }

            shared_state_unlock();
        }

        /* Log every 5 seconds */
        if (tick % 500 == 0) {
            shared_state_t s = {0};
            shared_state_snapshot(&s);
            ESP_LOGI(TAG, "t=%ds mode=%s trk=%d pitch=%d rec=%s play=%s loop=%d",
                     tick / 100,
                     s.mode == MODE_SYNTH ? "SYN" :
                     s.mode == MODE_DRUMS ? "DRM" : "OTH",
                     s.active_track,
                     s.tracks[0].pitch,
                     s.is_recording ? "Y" : "N",
                     s.is_playing ? "Y" : "N",
                     s.loop_length);
        }

        tick++;
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  AURA-SYNTH — Gestural Music Engine");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());

    /* PSRAM check */
    size_t psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    if (psram > 0) {
        ESP_LOGI(TAG, "PSRAM: %lu KB available", (unsigned long)(psram / 1024));
    } else {
        ESP_LOGW(TAG, "No PSRAM — delay and loop recorder disabled");
    }

    /* 1. Shared state */
    shared_state_init();
    ESP_LOGI(TAG, "[1/6] Shared state OK (%d bytes)", (int)sizeof(shared_state_t));

    /* 2. Block pool */
    if (!audio_pool_init()) {
        ESP_LOGE(TAG, "Block pool failed — aborting");
        return;
    }
    ESP_LOGI(TAG, "[2/6] Block pool OK");

    /* 3. I2S output */
    if (!i2s_output_init()) {
        ESP_LOGE(TAG, "I2S failed — aborting");
        return;
    }
    ESP_LOGI(TAG, "[3/6] I2S OK");

    /* 4. Sampler */
    sampler_init();
    // Uncomment when you add WAV files:
    sampler_register(0, "kick",  kick_wav_start,  kick_wav_end);
    sampler_register(1, "snare", snare_wav_start, snare_wav_end);
    sampler_register(2, "hihat", hihat_wav_start, hihat_wav_end);
    
    ESP_LOGI(TAG, "[4/6] Sampler OK");

    /* 5. Mixer + effects */
    mixer_init((float)SAMPLE_RATE);

    /* Optional: master delay effect (300ms, mild feedback) */
    if (psram > 0) {
        if (delay_init(&s_master_delay, 300.0f, 0.3f, 0.25f, (float)SAMPLE_RATE)) {
            mixer_set_master_fx(0, fx_delay, &s_master_delay);
            ESP_LOGI(TAG, "  Master delay: 300ms, feedback=0.3, mix=0.25");
        }
    }
    ESP_LOGI(TAG, "[5/6] Mixer + effects OK");

    /* 6. Loop recorder */
    if (psram > 0) {
        if (!loop_recorder_init()) {
            ESP_LOGW(TAG, "Loop recorder init failed — continuing without it");
        }
    }
    ESP_LOGI(TAG, "[6/6] Loop recorder OK");

    /* Report final memory state */
    ESP_LOGI(TAG, "Heap after init: %lu bytes",
             (unsigned long)esp_get_free_heap_size());
    if (psram > 0) {
        ESP_LOGI(TAG, "PSRAM after init: %lu KB",
                 (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    }

    /* ============ Launch tasks ============ */

    /* Audio pipeline — Core 1, highest priority */
    xTaskCreatePinnedToCore(
        audio_task_run, "audio", 8192, NULL,
        configMAX_PRIORITIES - 1, NULL, 1);

    /* Sensor/control — Core 0 (replace stub with real code) */
    xTaskCreatePinnedToCore(
        sensor_stub_task, "sensors", 4096, NULL,
        5, NULL, 0);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Pipeline running. Connect DAC.");
    ESP_LOGI(TAG, "  Stub: 0-10s synth, 10-20s drums,");
    ESP_LOGI(TAG, "        20-25s record, 25s+ playback");
    ESP_LOGI(TAG, "========================================");
}
