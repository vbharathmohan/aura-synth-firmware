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

/** Timeline position for live synth segment capture (0 if not recording). */
uint32_t loop_recorder_capture_time_us(void);

/** Loop length in µs (0 before first take completes). */
uint32_t loop_recorder_loop_length_us(void);

/** Playhead for sustained-synth lookup: playing head or paused position. */
uint32_t loop_recorder_playhead_us(void);

/**
 * If the loop is playing (or paused with a loop) and `track`'s tape has a
 * sustained synth segment covering `loop_pos_us`, writes pitch + volume and
 * returns true.
 */
bool loop_recorder_synth_playback_at(int track, uint32_t loop_pos_us,
                                     uint8_t *out_midi, float *out_vol);

#ifdef __cplusplus
}
#endif

#endif /* LOOP_RECORDER_H */
