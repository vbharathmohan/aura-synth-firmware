/**
 * loop_recorder.h — Gesture-event looper (records note events, not audio).
 *
 * Records live note_event_t triggers into per-track tapes and replays
 * those same note events back through g_note_queue.
 *
 * Prompt 2 transport:
 *   - record_pressed: toggle record/stop on active track
 *   - clear_pressed: clear active track
 *   - play_pause_pressed: toggle play/pause
 *
 * Loop-length rule:
 *   - First completed recording sets global loop length.
 *   - Later recordings fit into that fixed length.
 *   - If all tracks are cleared, loop length resets to zero.
 */

#ifndef LOOP_RECORDER_H
#define LOOP_RECORDER_H

#include "shared_state.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool loop_recorder_init(void);
void loop_recorder_deinit(void);

/** Feed a live user-triggered note event into the recorder. */
void loop_recorder_on_live_event(const note_event_t *evt);

/** Advance transport/playback and publish transport status into `snap`. */
void loop_recorder_update(shared_state_t *snap);

int loop_recorder_get_length(void);
bool loop_recorder_has_data(void);
void loop_recorder_clear_track(int track);
void loop_recorder_clear_all(void);

#ifdef __cplusplus
}
#endif

#endif /* LOOP_RECORDER_H */
