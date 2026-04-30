/**
 * shared_state.c — Default initialization for the cross-core shared state.
 */

#include "shared_state.h"
#include <string.h>
#include <assert.h>
#include "esp_log.h"

static const char *TAG = "shared_state";

shared_state_t    g_state;
SemaphoreHandle_t g_state_mutex = NULL;
QueueHandle_t     g_note_queue  = NULL;

void shared_state_init(void)
{
    g_state_mutex = xSemaphoreCreateMutex();
    assert(g_state_mutex != NULL);

    g_note_queue = xQueueCreate(NOTE_QUEUE_DEPTH, sizeof(note_event_t));
    assert(g_note_queue != NULL);

    memset(&g_state, 0, sizeof(g_state));

    /* Mode & transport.
     * MODE_DRUMS keeps the synth_voice rendering path disabled so the
     * mixer only spends CPU on the sampler — which is the only source
     * we use in the gesture instrument. */
    g_state.mode               = MODE_DRUMS;
    g_state.active_track       = 0;
    g_state.master_view        = false;

    /* ToF instrument selection */
    g_state.active_instrument  = INST_PIANO;
    g_state.pad_mode           = PAD_INSTRUMENTS;

    /* Per-track defaults (kept for future synth-voice tracks) */
    for (int i = 0; i < NUM_TRACKS; i++) {
        g_state.tracks[i].pitch         = 60;
        g_state.tracks[i].pitch_bend    = 0.0f;
        g_state.tracks[i].volume        = 0.0f;     /* silent — sampler-only */
        g_state.tracks[i].waveform_mix  = 0.5f;
        g_state.tracks[i].detune        = 0.0f;
        g_state.tracks[i].lfo_rate      = 0.01f;
        g_state.tracks[i].filter_cutoff = 2000.0f;
    }

    /* Master bus */
    g_state.master_volume     = 1.0f;
    g_state.master_filter     = 2000.0f;
    g_state.master_delay_mix  = 0.0f;          /* delay disabled by default */

    /* Loop recorder */
    g_state.is_recording = false;
    g_state.is_playing   = false;
    g_state.loop_length  = 0;
    g_state.playhead     = 0;

    ESP_LOGI(TAG, "shared_state_init: queue depth=%d, instrument=PIANO",
             NOTE_QUEUE_DEPTH);
}

bool note_event_post(uint8_t slot, uint8_t velocity, float speed,
                     bool loop, int8_t source)
{
    if (g_note_queue == NULL) return false;

    note_event_t evt = {
        .slot     = slot,
        .velocity = velocity,
        .speed    = speed,
        .loop     = loop,
        .source   = source,
    };
    /* Non-blocking: if the audio task is behind, dropping a note is
     * better than blocking the producer (sensor task on Core 1). */
    return xQueueSend(g_note_queue, &evt, 0) == pdTRUE;
}
