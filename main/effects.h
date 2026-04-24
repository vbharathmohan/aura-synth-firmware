/**
 * effects.h — Audio effects for the Aura-Synth pipeline.
 *
 * Every effect is a function: void (*effect_fn)(audio_block_t*, void* params)
 * Effects modify blocks in place. Chain them by calling one after another.
 *
 * Available effects:
 *   fx_volume   — simple gain
 *   fx_biquad   — biquad low-pass filter (per-track EQ)
 *   fx_delay    — echo/delay with feedback (PSRAM-backed buffer)
 *
 * USAGE:
 *   biquad_t filt;
 *   biquad_init_lpf(&filt, 800.0f, 0.707f, SAMPLE_RATE);
 *   fx_biquad(block, &filt);
 */

#ifndef EFFECTS_H
#define EFFECTS_H

#include "audio_block.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Effect function signature                                           */
/* ------------------------------------------------------------------ */

typedef void (*effect_fn)(audio_block_t *block, void *params);

/* ------------------------------------------------------------------ */
/* Volume                                                              */
/* ------------------------------------------------------------------ */

/** Simple gain effect. params = pointer to float (gain value). */
void fx_volume(audio_block_t *block, void *params);

/* ------------------------------------------------------------------ */
/* Biquad filter                                                       */
/* ------------------------------------------------------------------ */

typedef struct {
    /* Coefficients */
    float b0, b1, b2, a1, a2;
    /* State (left) */
    float x1L, x2L, y1L, y2L;
    /* State (right) */
    float x1R, x2R, y1R, y2R;
} biquad_t;

/**
 * Initialize as a low-pass filter.
 * @param cutoff_hz  Cutoff frequency
 * @param q          Resonance (0.707 = Butterworth, higher = resonant)
 * @param sample_rate Sample rate in Hz
 */
void biquad_init_lpf(biquad_t *f, float cutoff_hz, float q, float sample_rate);

/**
 * Update cutoff frequency without resetting filter state.
 * Call this when the user changes the filter knob.
 */
void biquad_update_cutoff(biquad_t *f, float cutoff_hz, float q, float sample_rate);

/** Process a block through the biquad filter. */
void fx_biquad(audio_block_t *block, void *params);

/* ------------------------------------------------------------------ */
/* Delay                                                               */
/* ------------------------------------------------------------------ */

typedef struct {
    int32_t *buf_L;          /* ring buffer left (PSRAM) */
    int32_t *buf_R;          /* ring buffer right (PSRAM) */
    size_t   buf_len;        /* buffer length in samples */
    size_t   write_pos;      /* current write position */
    float    feedback;       /* 0.0 - 0.95 (higher = more repeats) */
    float    mix;            /* 0.0 - 1.0 (dry/wet) */
    bool     initialized;
} delay_t;

/**
 * Initialize delay effect. Allocates ring buffers in PSRAM.
 * @param delay_ms    Delay time in milliseconds
 * @param feedback    Feedback amount (0.0 - 0.95)
 * @param mix         Dry/wet mix (0.0 = dry, 1.0 = fully wet)
 * @param sample_rate Sample rate in Hz
 * @return            true if PSRAM allocation succeeded
 */
bool delay_init(delay_t *d, float delay_ms, float feedback,
                float mix, float sample_rate);

/** Free delay buffers. */
void delay_deinit(delay_t *d);

/** Process a block through the delay. */
void fx_delay(audio_block_t *block, void *params);

#ifdef __cplusplus
}
#endif

#endif /* EFFECTS_H */
