/**
 * loop_recorder.h — Gesture tape recorder for Aura-Synth.
 *
 * Records control_frame_t snapshots at a fixed control rate (100 Hz).
 * Each track has its own tape. When playing back, recorded frames
 * are applied to the shared state as if the user were performing.
 *
 * Flow:
 *   1. User presses RECORD → loop_recorder starts capturing frames
 *   2. User presses RECORD again → loop length is set, playback starts
 *   3. User can overdub on other tracks while the loop plays
 *   4. User presses CLEAR → active track's tape is erased
 *
 * Tape storage is in PSRAM (each tape = MAX_LOOP_FRAMES × 8 bytes).
 *
 * USAGE:
 *   loop_recorder_init();
 *
 *   // Each control cycle (100 Hz):
 *   loop_recorder_update(&state_snapshot);
 *
 *   // The recorder reads record_pressed / clear_pressed from state
 *   // and writes is_recording / is_playing / playhead / loop_length back.
 */

#ifndef LOOP_RECORDER_H
#define LOOP_RECORDER_H

#include "shared_state.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the loop recorder. Allocates tape buffers in PSRAM.
 * @return true if PSRAM allocation succeeded for all tracks
 */
bool loop_recorder_init(void);

/**
 * Free all tape buffers.
 */
void loop_recorder_deinit(void);

/**
 * Process one control cycle. Call this at the control rate (e.g. 100 Hz)
 * from the audio task or a dedicated control task.
 *
 * This function:
 *   - Handles record/stop/clear button presses
 *   - Records the current frame if recording
 *   - Plays back recorded frames if playing
 *   - Updates the shared state (is_recording, is_playing, playhead, etc.)
 *
 * @param snap  Current shared state snapshot (read/write)
 */
void loop_recorder_update(shared_state_t *snap);

/**
 * Get the current loop length in frames (0 = no loop).
 */
int loop_recorder_get_length(void);

/**
 * Check if any track has recorded data.
 */
bool loop_recorder_has_data(void);

/**
 * Clear a specific track's tape.
 */
void loop_recorder_clear_track(int track);

/**
 * Clear all tapes.
 */
void loop_recorder_clear_all(void);

#ifdef __cplusplus
}
#endif

#endif /* LOOP_RECORDER_H */
