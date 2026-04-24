/**
 * mixer.h — Multi-track audio mixer for Aura-Synth.
 *
 * Pipeline per cycle:
 *   For each track:
 *     1. Render source (synth voice or sampler) into a block
 *     2. Apply per-track effects chain (up to MAX_TRACK_FX)
 *     3. Accumulate into the mix bus
 *   Then:
 *     4. Apply master effects chain (up to MAX_MASTER_FX)
 *     5. Return the final mixed block
 *
 * The mixer owns the synth voices and manages their lifecycle.
 * The sampler is global (shared across all modes).
 *
 * USAGE:
 *   mixer_init();
 *   // In audio loop:
 *   audio_block_t *out = mixer_process(&state_snapshot);
 *   audio_block_to_i2s(out, i2s_buf, 1);
 *   audio_free(out);
 */

#ifndef MIXER_H
#define MIXER_H

#include "audio_block.h"
#include "shared_state.h"
#include "synth_voice.h"
#include "effects.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

#define MAX_TRACK_FX    4    /* effects slots per track */
#define MAX_MASTER_FX   4    /* effects slots on master bus */

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * Initialize the mixer: synth voices, effects chains.
 * Call once at boot after audio_pool_init().
 * @param sample_rate  Audio sample rate (e.g. 44100)
 * @return true on success
 */
bool mixer_init(float sample_rate);

/**
 * Set a per-track effect.
 * @param track   Track index (0 to NUM_TRACKS-1)
 * @param slot    Effect slot (0 to MAX_TRACK_FX-1)
 * @param fn      Effect function (or NULL to clear)
 * @param params  Parameter struct for the effect
 */
void mixer_set_track_fx(int track, int slot, effect_fn fn, void *params);

/**
 * Set a master bus effect.
 * @param slot    Effect slot (0 to MAX_MASTER_FX-1)
 * @param fn      Effect function (or NULL to clear)
 * @param params  Parameter struct for the effect
 */
void mixer_set_master_fx(int slot, effect_fn fn, void *params);

/**
 * Run the full mix pipeline for one audio cycle.
 * Reads mode, track params, and triggers from the state snapshot.
 * Returns a mixed block (caller must audio_free() it).
 * Returns NULL if block allocation fails.
 */
audio_block_t *mixer_process(const shared_state_t *snap);

/**
 * Get a pointer to a track's synth voice (for direct control).
 */
synth_voice_t *mixer_get_voice(int track);

/**
 * Get the per-track biquad filter (for cutoff updates).
 */
biquad_t *mixer_get_track_filter(int track);

#ifdef __cplusplus
}
#endif

#endif /* MIXER_H */
