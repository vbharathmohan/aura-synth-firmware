/**
 * shared_state.c — Default initialization for the cross-core shared state.
 */

#include "shared_state.h"
#include <string.h>
#include <assert.h>

shared_state_t    g_state;
SemaphoreHandle_t g_state_mutex = NULL;

void shared_state_init(void)
{
    g_state_mutex = xSemaphoreCreateMutex();
    assert(g_state_mutex != NULL);

    memset(&g_state, 0, sizeof(g_state));

    /* Mode & transport */
    g_state.mode         = MODE_SYNTH;
    g_state.active_track = 0;
    g_state.master_view  = false;

    /* Per-track defaults */
    for (int i = 0; i < NUM_TRACKS; i++) {
        g_state.tracks[i].pitch         = 60;       /* middle C */
        g_state.tracks[i].pitch_bend    = 0.0f;
        g_state.tracks[i].volume        = 0.8f;
        g_state.tracks[i].waveform_mix  = 0.5f;     /* half sine, half saw */
        g_state.tracks[i].detune        = 0.0f;
        g_state.tracks[i].lfo_rate      = 5.0f;     /* 5 Hz tremolo */
        g_state.tracks[i].filter_cutoff = 2000.0f;  /* wide open */
    }

    /* Master bus */
    g_state.master_volume = 1.0f;
    g_state.master_filter = 2000.0f;

    /* Loop recorder */
    g_state.is_recording = false;
    g_state.is_playing   = false;
    g_state.loop_length  = 0;
    g_state.playhead     = 0;
}
