/**
 * sampler.h — Flash-embedded WAV sample player for Aura-Synth.
 *
 * Plays WAV samples stored in firmware flash via EMBED_FILES.
 * Supports:
 *   - Multiple sample slots (drums: 8, piano base, synth pad, etc.)
 *   - Per-trigger velocity (volume scaling)
 *   - Pitch shifting via playback speed
 *   - One-shot and looping modes
 *   - Polyphonic voice stealing (multiple simultaneous playbacks)
 *
 * Samples are accessed via direct pointer — zero-copy from flash.
 * No malloc, no filesystem, no latency.
 *
 * USAGE:
 *   // At boot: register embedded samples
 *   sampler_init();
 *   sampler_register(0, "kick", kick_wav_start, kick_wav_end);
 *   sampler_register(1, "snare", snare_wav_start, snare_wav_end);
 *
 *   // On trigger:
 *   sampler_trigger(0, 200, 1.0f, false);  // slot 0, velocity 200, normal pitch, one-shot
 *
 *   // In audio task:
 *   audio_block_t *blk = audio_alloc();
 *   audio_block_clear(blk);
 *   sampler_render(blk);  // mixes all active voices into the block
 */

#ifndef SAMPLER_H
#define SAMPLER_H

#include <stdint.h>
#include <stdbool.h>
#include "audio_block.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

#define SAMPLER_MAX_SLOTS   16   /* max registered samples */
#define SAMPLER_MAX_VOICES  8    /* max simultaneous playbacks */
#define SAMPLER_NAME_LEN    16

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */

/** A registered sample (pointer into flash) */
typedef struct {
    const int16_t *data;         /* PCM data (after 44-byte WAV header) */
    uint32_t       length;       /* number of samples */
    uint32_t       sample_rate;  /* original sample rate */
    char           name[SAMPLER_NAME_LEN];
    bool           loaded;
} sample_slot_t;

/** An active playback instance */
typedef struct {
    const sample_slot_t *slot;
    float    position;           /* fractional sample index */
    float    speed;              /* playback speed (1.0 = normal) */
    float    volume;             /* 0.0 - 1.0 */
    bool     active;
    bool     looping;
} sampler_voice_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** Initialize the sampler. Call once at boot. */
void sampler_init(void);

/**
 * Register a flash-embedded WAV sample.
 *
 * @param slot_idx   Index 0 to SAMPLER_MAX_SLOTS-1
 * @param name       Short name for logging (e.g. "kick")
 * @param wav_start  Pointer from asm("_binary_xxx_wav_start")
 * @param wav_end    Pointer from asm("_binary_xxx_wav_end")
 * @return           true if the WAV header is valid and sample registered
 */
bool sampler_register(int slot_idx, const char *name,
                      const uint8_t *wav_start, const uint8_t *wav_end);

/**
 * Trigger a sample.
 *
 * @param slot_idx   Which sample to play
 * @param velocity   0-255, mapped to volume
 * @param speed      Playback speed (1.0 = normal, 2.0 = octave up)
 * @param loop       true to loop, false for one-shot
 * @return           Voice index (0 to MAX_VOICES-1), or -1 if failed
 */
int sampler_trigger(int slot_idx, uint8_t velocity, float speed, bool loop);

/**
 * Stop a specific voice.
 */
void sampler_stop_voice(int voice_idx);

/**
 * Stop all active voices.
 */
void sampler_stop_all(void);

/**
 * Render all active voices into a block (accumulates — does NOT clear).
 * Call from the audio task each cycle.
 */
void sampler_render(audio_block_t *blk);

/**
 * Get the number of currently active voices.
 */
int sampler_active_count(void);

/**
 * Get a registered slot (for debug/UI). Returns NULL if not loaded.
 */
const sample_slot_t *sampler_get_slot(int slot_idx);

#ifdef __cplusplus
}
#endif

#endif /* SAMPLER_H */
