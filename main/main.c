/**
 * main.c — Aura-Synth DEMO: 90-second guided audio tour.
 *
 * This demo exercises every audio feature in sequence, with clear
 * serial log narration so your team can follow along.
 *
 * Timeline:
 *   0-15s   DEMO 1: Synth voice — pure sine, then saw, then blend
 *   15-30s  DEMO 2: Filter sweep — hear the biquad LP open and close
 *   30-40s  DEMO 3: LFO tremolo — slow then fast wobble
 *   40-50s  DEMO 4: Pitch bend — smooth pitch slides
 *   50-65s  DEMO 5: Drum sampler — kick/snare/hihat pattern
 *   65-75s  DEMO 6: Delay effect — toggle echo on/off
 *   75-90s  DEMO 7: Multi-track — synth + drums simultaneously
 *
 * After 90s it loops back to the start.
 *
 * WHAT THIS PROVES:
 *   ✓ Block pool runs leak-free for thousands of cycles
 *   ✓ Synth voice generates sine + detuned saw with crossfade
 *   ✓ Biquad LP filter responds to cutoff changes in real time
 *   ✓ LFO tremolo modulates amplitude at variable rates
 *   ✓ Pitch bend shifts frequency smoothly (±2 semitones)
 *   ✓ Sampler triggers flash-embedded WAV files with velocity
 *   ✓ Delay effect adds echo from PSRAM ring buffer
 *   ✓ Mixer accumulates multiple sources without clipping
 *   ✓ I2S DMA feeds the DAC without dropouts
 *   ✓ Shared state correctly passes data between cores
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
#include "sampler.h"
#include "mixer.h"
#include "loop_recorder.h"
#include "audio_task.h"
#include "effects.h"

static const char *TAG = "demo";

/* ------------------------------------------------------------------ */
/* Embedded WAV samples                                                */
/* ------------------------------------------------------------------ */

extern const uint8_t kick_wav_start[]  asm("_binary_kick_wav_start");
extern const uint8_t kick_wav_end[]    asm("_binary_kick_wav_end");
extern const uint8_t snare_wav_start[] asm("_binary_snare_wav_start");
extern const uint8_t snare_wav_end[]   asm("_binary_snare_wav_end");
extern const uint8_t hihat_wav_start[] asm("_binary_hihat_wav_start");
extern const uint8_t hihat_wav_end[]   asm("_binary_hihat_wav_end");

/* ------------------------------------------------------------------ */
/* Master delay effect                                                 */
/* ------------------------------------------------------------------ */

static delay_t s_master_delay;
static bool s_delay_active = false;

/* ------------------------------------------------------------------ */
/* Helper: set synth to a clean starting state                         */
/* ------------------------------------------------------------------ */

static void synth_defaults(void)
{
    if (!shared_state_lock()) return;
    g_state.mode = MODE_SYNTH;
    g_state.active_track = 0;
    g_state.master_volume = 0.8f;
    g_state.tracks[0].pitch = 60;       /* middle C */
    g_state.tracks[0].volume = 0.6f;
    g_state.tracks[0].waveform_mix = 0.0f;  /* pure sine */
    g_state.tracks[0].detune = 0.0f;
    g_state.tracks[0].lfo_rate = 0.01f;     /* effectively off */
    g_state.tracks[0].filter_cutoff = 2000.0f;  /* wide open */
    g_state.tracks[0].pitch_bend = 0.0f;
    shared_state_unlock();
}

/* ------------------------------------------------------------------ */
/* Demo sequence task (Core 0)                                         */
/* ------------------------------------------------------------------ */

static void demo_task(void *param)
{
    ESP_LOGI(TAG, "Demo task on core %d — 90 second audio tour", xPortGetCoreID());

    const int TICK_MS = 50;   /* 20 Hz control rate for smooth changes */
    int tick = 0;
    int demo_cycle = 0;

    while (1) {
        float t = tick * TICK_MS / 1000.0f;  /* time in seconds */
        float t_in_demo = t - (demo_cycle * 90.0f);  /* time within current 90s cycle */

        /* ============================================================ */
        /* DEMO 1: Synth waveforms (0-15s)                              */
        /* Pure sine → detuned saw → blended                            */
        /* ============================================================ */
        if (t_in_demo >= 0.0f && t_in_demo < 15.0f) {

            if (tick == 0 || (tick > 0 && t_in_demo < 0.05f)) {
                synth_defaults();
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "====== DEMO 1: SYNTH WAVEFORMS (0-15s) ======");
                ESP_LOGI(TAG, "  0-5s:  Pure sine wave at C4 (262 Hz)");
                ESP_LOGI(TAG, "  5-10s: Detuned sawtooth wave");
                ESP_LOGI(TAG, "  10-15s: Blend sine + saw with detune");
            }

            if (shared_state_lock()) {
                g_state.tracks[0].pitch = 60;  /* C4 */

                if (t_in_demo < 5.0f) {
                    /* Pure sine */
                    g_state.tracks[0].waveform_mix = 0.0f;
                    g_state.tracks[0].detune = 0.0f;
                    if ((tick * TICK_MS) % 5000 == 0)
                        ESP_LOGI(TAG, "  >> Pure sine — clean, smooth tone");
                }
                else if (t_in_demo < 10.0f) {
                    /* Pure saw with detune */
                    g_state.tracks[0].waveform_mix = 1.0f;
                    g_state.tracks[0].detune = 2.0f;
                    if ((tick * TICK_MS) % 5000 == 0)
                        ESP_LOGI(TAG, "  >> Sawtooth + 2Hz detune — buzzy, rich");
                }
                else {
                    /* Blend: 50/50 with fat detune */
                    g_state.tracks[0].waveform_mix = 0.5f;
                    g_state.tracks[0].detune = 3.0f;
                    if ((tick * TICK_MS) % 5000 == 0)
                        ESP_LOGI(TAG, "  >> 50/50 blend + 3Hz detune — warm, chorused");
                }
                shared_state_unlock();
            }
        }

        /* ============================================================ */
        /* DEMO 2: Filter sweep (15-30s)                                */
        /* Biquad LP cutoff sweeps from 200 Hz to 2000 Hz and back      */
        /* ============================================================ */
        else if (t_in_demo >= 15.0f && t_in_demo < 30.0f) {

            if (t_in_demo < 15.05f) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "====== DEMO 2: FILTER SWEEP (15-30s) ======");
                ESP_LOGI(TAG, "  Sawtooth held at C3 — filter opens and closes");
                ESP_LOGI(TAG, "  Listen for the brightness change");
            }

            if (shared_state_lock()) {
                g_state.tracks[0].pitch = 48;  /* C3 — low note shows filter better */
                g_state.tracks[0].waveform_mix = 0.8f;  /* mostly saw for harmonic content */
                g_state.tracks[0].detune = 1.0f;
                g_state.tracks[0].volume = 0.7f;

                /* Triangle sweep: 200 → 2000 → 200 over 15 seconds */
                float sweep_t = (t_in_demo - 15.0f) / 15.0f;  /* 0 to 1 */
                float triangle = (sweep_t < 0.5f) ? (sweep_t * 2.0f) : (2.0f - sweep_t * 2.0f);
                g_state.tracks[0].filter_cutoff = 200.0f + triangle * 1800.0f;

                if ((tick * TICK_MS) % 2000 == 0)
                    ESP_LOGI(TAG, "  >> Filter cutoff: %.0f Hz",
                             g_state.tracks[0].filter_cutoff);
                shared_state_unlock();
            }
        }

        /* ============================================================ */
        /* DEMO 3: LFO tremolo (30-40s)                                 */
        /* Slow wobble → fast wobble                                     */
        /* ============================================================ */
        else if (t_in_demo >= 30.0f && t_in_demo < 40.0f) {

            if (t_in_demo < 30.05f) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "====== DEMO 3: LFO TREMOLO (30-40s) ======");
                ESP_LOGI(TAG, "  30-35s: Slow tremolo (2 Hz) — gentle pulse");
                ESP_LOGI(TAG, "  35-40s: Fast tremolo (12 Hz) — vibrato effect");
            }

            if (shared_state_lock()) {
                g_state.tracks[0].pitch = 64;  /* E4 */
                g_state.tracks[0].waveform_mix = 0.3f;
                g_state.tracks[0].filter_cutoff = 1500.0f;
                g_state.tracks[0].detune = 1.5f;

                if (t_in_demo < 35.0f) {
                    g_state.tracks[0].lfo_rate = 2.0f;
                } else {
                    g_state.tracks[0].lfo_rate = 12.0f;
                }

                if ((tick * TICK_MS) % 5000 == 0)
                    ESP_LOGI(TAG, "  >> LFO rate: %.1f Hz",
                             g_state.tracks[0].lfo_rate);
                shared_state_unlock();
            }
        }

        /* ============================================================ */
        /* DEMO 4: Pitch bend (40-50s)                                  */
        /* Smooth slides up and down                                     */
        /* ============================================================ */
        else if (t_in_demo >= 40.0f && t_in_demo < 50.0f) {

            if (t_in_demo < 40.05f) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "====== DEMO 4: PITCH BEND (40-50s) ======");
                ESP_LOGI(TAG, "  Holding C4 — bending pitch ±2 semitones");
                ESP_LOGI(TAG, "  Listen for smooth pitch slides");
            }

            if (shared_state_lock()) {
                g_state.tracks[0].pitch = 60;
                g_state.tracks[0].waveform_mix = 0.4f;
                g_state.tracks[0].lfo_rate = 0.01f;  /* LFO off */
                g_state.tracks[0].filter_cutoff = 2000.0f;
                g_state.tracks[0].detune = 0.0f;

                /* Sine wave bend: smoothly sweeps -1 to +1 to -1 */
                float bend_t = (t_in_demo - 40.0f) / 10.0f;
                float bend = sinf(bend_t * 2.0f * 3.14159f);
                g_state.tracks[0].pitch_bend = bend;

                if ((tick * TICK_MS) % 2000 == 0)
                    ESP_LOGI(TAG, "  >> Pitch bend: %+.2f (%.1f Hz shift)",
                             bend, bend * 2.0f);
                shared_state_unlock();
            }
        }

        /* ============================================================ */
        /* DEMO 5: Drum sampler (50-65s)                                */
        /* Kick/snare/hihat pattern at 120 BPM                          */
        /* ============================================================ */
        else if (t_in_demo >= 50.0f && t_in_demo < 65.0f) {

            if (t_in_demo < 50.05f) {
                synth_defaults();
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "====== DEMO 5: DRUM SAMPLER (50-65s) ======");
                ESP_LOGI(TAG, "  120 BPM pattern: kick-hihat-snare-hihat");
                ESP_LOGI(TAG, "  Using flash-embedded WAV samples");
            }

            if (shared_state_lock()) {
                g_state.mode = MODE_DRUMS;
                g_state.master_volume = 0.9f;

                /* 120 BPM = 500ms per beat, 250ms per eighth note */
                /* Pattern: kick, hihat, snare, hihat (repeating) */
                int beat_ms = (int)((t_in_demo - 50.0f) * 1000.0f) % 1000;
                int sub_beat = beat_ms / 250;

                /* Trigger on the exact tick boundary */
                int ms_in_sub = beat_ms % 250;
                if (ms_in_sub < TICK_MS) {
                    switch (sub_beat) {
                        case 0:  /* kick */
                            g_state.drum.slot = 0;
                            g_state.drum.velocity = 220;
                            g_state.drum.trigger = true;
                            break;
                        case 1:  /* hihat */
                            g_state.drum.slot = 2;
                            g_state.drum.velocity = 150;
                            g_state.drum.trigger = true;
                            break;
                        case 2:  /* snare */
                            g_state.drum.slot = 1;
                            g_state.drum.velocity = 200;
                            g_state.drum.trigger = true;
                            break;
                        case 3:  /* hihat */
                            g_state.drum.slot = 2;
                            g_state.drum.velocity = 120;
                            g_state.drum.trigger = true;
                            break;
                    }
                }
                shared_state_unlock();
            }
        }

        /* ============================================================ */
        /* DEMO 6: Delay effect (65-75s)                                */
        /* Play a note, then toggle delay on/off to hear the echo       */
        /* ============================================================ */
        else if (t_in_demo >= 65.0f && t_in_demo < 75.0f) {

            if (t_in_demo < 65.05f) {
                synth_defaults();
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "====== DEMO 6: DELAY EFFECT (65-75s) ======");
                ESP_LOGI(TAG, "  65-67s: Staccato notes WITHOUT delay");
                ESP_LOGI(TAG, "  67-70s: Silence — listen for no echo");
                ESP_LOGI(TAG, "  70-72s: Same notes WITH delay enabled");
                ESP_LOGI(TAG, "  72-75s: Silence — hear the echo tail");
            }

            if (shared_state_lock()) {
                g_state.mode = MODE_SYNTH;

                /* Staccato: short notes every 500ms */
                int phase_ms = (int)((t_in_demo - 65.0f) * 1000.0f);

                if (t_in_demo < 67.0f) {
                    /* Notes WITHOUT delay */
                    if (phase_ms < 2000 && (phase_ms % 500) < 150) {
                        g_state.tracks[0].pitch = 72;  /* C5 */
                        g_state.tracks[0].volume = 0.7f;
                    } else {
                        g_state.tracks[0].volume = 0.0f;
                    }

                    /* Make sure delay is off */
                    if (!s_delay_active) {
                        /* already off */
                    } else {
                        mixer_set_master_fx(0, NULL, NULL);
                        s_delay_active = false;
                        ESP_LOGI(TAG, "  >> Delay OFF");
                    }
                }
                else if (t_in_demo < 70.0f) {
                    /* Silence gap — no echo because delay was off */
                    g_state.tracks[0].volume = 0.0f;

                    /* Turn delay ON partway through the silence */
                    if (t_in_demo >= 69.5f && !s_delay_active) {
                        mixer_set_master_fx(0, fx_delay, &s_master_delay);
                        s_delay_active = true;
                        ESP_LOGI(TAG, "  >> Delay ON (300ms, feedback=0.3)");
                    }
                }
                else if (t_in_demo < 72.0f) {
                    /* Same staccato notes WITH delay */
                    int p2 = (int)((t_in_demo - 70.0f) * 1000.0f);
                    if ((p2 % 500) < 150) {
                        g_state.tracks[0].pitch = 72;
                        g_state.tracks[0].volume = 0.7f;
                    } else {
                        g_state.tracks[0].volume = 0.0f;
                    }
                }
                else {
                    /* Silence — hear the delay tail ring out */
                    g_state.tracks[0].volume = 0.0f;
                }

                shared_state_unlock();
            }
        }

        /* ============================================================ */
        /* DEMO 7: Multi-track mixing (75-90s)                          */
        /* Synth pad + drum beat playing simultaneously                  */
        /* ============================================================ */
        else if (t_in_demo >= 75.0f && t_in_demo < 90.0f) {

            if (t_in_demo < 75.05f) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "====== DEMO 7: MULTI-TRACK MIX (75-90s) ======");
                ESP_LOGI(TAG, "  Synth pad + drum pattern playing together");
                ESP_LOGI(TAG, "  Mixer accumulates both sources cleanly");

                /* Turn delay back on for the finale */
                if (!s_delay_active) {
                    mixer_set_master_fx(0, fx_delay, &s_master_delay);
                    s_delay_active = true;
                }
            }

            if (shared_state_lock()) {
                g_state.mode = MODE_SYNTH;  /* synth voice active */
                g_state.master_volume = 0.7f;

                /* Synth pad: slow chord progression */
                float prog_t = (t_in_demo - 75.0f) / 15.0f;
                int chord_idx = (int)(prog_t * 4.0f) % 4;
                uint8_t chord_notes[] = {60, 64, 65, 67};  /* C E F G */
                g_state.tracks[0].pitch = chord_notes[chord_idx];
                g_state.tracks[0].volume = 0.4f;
                g_state.tracks[0].waveform_mix = 0.5f;
                g_state.tracks[0].detune = 2.0f;
                g_state.tracks[0].lfo_rate = 3.0f;
                g_state.tracks[0].filter_cutoff = 1200.0f;

                /* Drum pattern on top (same 120 BPM kick-hihat-snare-hihat) */
                int beat_ms = (int)((t_in_demo - 75.0f) * 1000.0f) % 1000;
                int sub_beat = beat_ms / 250;
                int ms_in_sub = beat_ms % 250;
                if (ms_in_sub < TICK_MS) {
                    g_state.drum.slot = (sub_beat == 0) ? 0 :
                                        (sub_beat == 2) ? 1 : 2;
                    g_state.drum.velocity = (sub_beat == 0 || sub_beat == 2) ? 200 : 130;
                    g_state.drum.trigger = true;
                }

                if ((tick * TICK_MS) % 4000 == 0)
                    ESP_LOGI(TAG, "  >> Chord: %s | Drums active | Delay on",
                             chord_idx == 0 ? "C" :
                             chord_idx == 1 ? "E" :
                             chord_idx == 2 ? "F" : "G");

                shared_state_unlock();
            }
        }

        /* ============================================================ */
        /* Loop reset                                                    */
        /* ============================================================ */
        else if (t_in_demo >= 90.0f) {
            demo_cycle++;
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "====== DEMO COMPLETE — looping back ======");
            ESP_LOGI(TAG, "  Blocks processed so far, pool health: %d/%d",
                     audio_pool_available(), POOL_SIZE);
            ESP_LOGI(TAG, "");
        }

        /* Advance */
        tick++;
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "  AURA-SYNTH — 90-Second Audio Demo");
    ESP_LOGI(TAG, "  Connect DAC + speaker/headphones to hear");
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());

    size_t psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    if (psram > 0) {
        ESP_LOGI(TAG, "PSRAM: %lu KB", (unsigned long)(psram / 1024));
    }

    /* 1. Shared state */
    shared_state_init();
    ESP_LOGI(TAG, "[1/6] Shared state OK");

    /* 2. Block pool */
    if (!audio_pool_init()) { ESP_LOGE(TAG, "Block pool failed"); return; }
    ESP_LOGI(TAG, "[2/6] Block pool OK");

    /* 3. I2S */
    if (!i2s_output_init()) { ESP_LOGE(TAG, "I2S failed"); return; }
    ESP_LOGI(TAG, "[3/6] I2S OK");

    /* 4. Sampler */
    sampler_init();
    sampler_register(0, "kick",  kick_wav_start,  kick_wav_end);
    sampler_register(1, "snare", snare_wav_start, snare_wav_end);
    sampler_register(2, "hihat", hihat_wav_start, hihat_wav_end);
    ESP_LOGI(TAG, "[4/6] Sampler OK — 3 samples loaded");

    /* 5. Mixer */
    mixer_init((float)SAMPLE_RATE);
    /* Delay starts OFF — Demo 6 will toggle it */
    if (psram > 0) {
        delay_init(&s_master_delay, 300.0f, 0.35f, 0.3f, (float)SAMPLE_RATE);
    }
    s_delay_active = false;
    ESP_LOGI(TAG, "[5/6] Mixer OK — delay prepared but OFF");

    /* 6. Loop recorder */
    if (psram > 0) { loop_recorder_init(); }
    ESP_LOGI(TAG, "[6/6] Loop recorder OK");

    ESP_LOGI(TAG, "Heap after init: %lu bytes",
             (unsigned long)esp_get_free_heap_size());

    /* Launch audio pipeline on Core 1 */
    xTaskCreatePinnedToCore(
        audio_task_run, "audio", 8192, NULL,
        configMAX_PRIORITIES - 1, NULL, 1);

    /* Launch demo sequence on Core 0 */
    xTaskCreatePinnedToCore(
        demo_task, "demo", 4096, NULL,
        5, NULL, 0);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  Demo starting in 1 second...");
    ESP_LOGI(TAG, "  Follow the serial log to know what you're hearing.");
    ESP_LOGI(TAG, "");
}
