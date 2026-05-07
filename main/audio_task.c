/**
 * audio_task.c — Core 1 audio pipeline runner.
 *
 * Runs the complete pipeline at ~172 Hz (44100 / 256 = 172 blocks/sec).
 * The loop recorder runs at a sub-divided control rate (~100 Hz).
 */

#include "audio_task.h"

#include "shared_state.h"
#include "audio_block.h"
#include "i2s_output.h"
#include "mixer.h"
#include "sampler.h"
#include "loop_recorder.h"
#include "effects.h"

#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "audio_task";

/* Control rate divider: run loop_recorder at ~100 Hz.
 * At 44100 Hz / 256 samples = ~172 blocks/sec.
 * Every 2 blocks ≈ 86 Hz control rate — close enough. */
#define CONTROL_DIVIDER 2

void audio_task_run(void *param)
{
    ESP_LOGI(TAG, "Audio task started on core %d", xPortGetCoreID());

    /* DMA-capable I2S output buffer */
    int16_t *i2s_buf = (int16_t *)heap_caps_malloc(
        BLOCK_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_DMA);

    if (i2s_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer — aborting audio task");
        vTaskDelete(NULL);
        return;
    }

    shared_state_t snap = {0};
    int block_count = 0;
    int control_counter = 0;

    /* Transport press flags accumulate across audio cycles. shared_state_snapshot()
     * consumes the global flags every call (~172 Hz), but loop_recorder_update()
     * only runs once per CONTROL_DIVIDER blocks (~86 Hz). Without this latch,
     * presses that land on a non-control cycle would be silently dropped. */
    bool pending_record = false;
    bool pending_clear = false;
    bool pending_play_pause = false;

    while (1) {
        /* 1. Snapshot shared state (consumes one-shot triggers) */
        shared_state_snapshot(&snap);

        /* Latch transport edges from this cycle so they survive until
         * the next loop_recorder_update() call. */
        if (snap.record_pressed)     pending_record = true;
        if (snap.clear_pressed)      pending_clear = true;
        if (snap.play_pause_pressed) pending_play_pause = true;

        /* 2. Drain the note-event queue. Producers are sensor_task,
         * the integration-mode button task, and (later) the loop
         * recorder.  Each event becomes a sampler_trigger() call.
         * We bound the work per cycle so an over-eager producer
         * can't starve the audio loop. */
        if (g_note_queue != NULL) {
            note_event_t evt;
            int drained = 0;
            while (drained < NOTE_QUEUE_DEPTH &&
                   xQueueReceive(g_note_queue, &evt, 0) == pdTRUE) {
                /* Recorder captures only live user events (not loop playback). */
                loop_recorder_on_live_event(&evt);

                /* Sustained synth segments are tape-driven in sensor_task; only
                 * the recorder ingests these from the queue (no sampler). */
                if (evt.source == NOTE_SOURCE_SYNTH_NOTE && evt.duration_us > 0) {
                    /* no-op */
                } else {
                    float spd = evt.speed * shared_state_sample_speed_scale(&snap);
                    sampler_trigger(evt.slot, evt.velocity, spd, evt.loop);
                }
                drained++;
            }
        }

        /* 3. Run loop recorder at control rate */
        control_counter++;
        if (control_counter >= CONTROL_DIVIDER) {
            control_counter = 0;

            /* Hand the latched press edges to the recorder. Clear the
             * latches only after the recorder has had a chance to act. */
            snap.record_pressed     = pending_record;
            snap.clear_pressed      = pending_clear;
            snap.play_pause_pressed = pending_play_pause;
            pending_record = false;
            pending_clear = false;
            pending_play_pause = false;

            loop_recorder_update(&snap);

            /* Write transport state back to global shared state */
            if (shared_state_lock()) {
                g_state.is_recording = snap.is_recording;
                g_state.is_playing   = snap.is_playing;
                g_state.loop_length  = snap.loop_length;
                g_state.playhead     = snap.playhead;
                shared_state_unlock();
            }
        }

        /* 4. Run the mixer pipeline → returns a mixed block */
        audio_block_t *out = mixer_process(&snap);

        if (out != NULL) {
            /* 5. Convert int32 → interleaved int16 for I2S */
            audio_block_to_i2s(out, i2s_buf, 1);
            audio_free(out);

            /* 6. Write to I2S DMA (blocks until buffer available) */
            i2s_output_write(i2s_buf, BLOCK_SAMPLES * 2 * sizeof(int16_t));
        } else {
            /* Pool exhausted — write silence to prevent DMA underrun */
            memset(i2s_buf, 0, BLOCK_SAMPLES * 2 * sizeof(int16_t));
            i2s_output_write(i2s_buf, BLOCK_SAMPLES * 2 * sizeof(int16_t));
        }

        /* Periodic diagnostics */
        block_count++;
        if (block_count % 1000 == 0) {
            const char *inst_name =
                snap.active_instrument == INST_PIANO      ? "piano" :
                snap.active_instrument == INST_STEEL_DRUM ? "steel" :
                snap.active_instrument == INST_TRUMPET    ? "trump" :
                snap.active_instrument == INST_808_BASS   ? "808"   : "?";
            ESP_LOGI(TAG,
                "Blk=%d | Pool=%d/%d | Sampler=%d | Inst=%s | "
                "Rec=%s Play=%s Loop=%d Head=%d",
                block_count,
                audio_pool_available(), POOL_SIZE,
                sampler_active_count(),
                inst_name,
                snap.is_recording ? "Y" : "N",
                snap.is_playing ? "Y" : "N",
                snap.loop_length,
                snap.playhead);
        }
    }
}
