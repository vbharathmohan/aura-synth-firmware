/**
 * audio_block.h — Fixed-size audio blocks and zero-allocation pool.
 *
 * Every stage in the audio pipeline operates on audio_block_t:
 *   [Track players] → [Per-track FX] → [Mixer] → [Master FX] → [I2S]
 *
 * Blocks use int32_t internally for mixing headroom. When you sum 4 tracks
 * at full volume, int16 overflows. We downscale to int16 only at the very
 * end, right before writing to I2S DMA.
 *
 * The pool pre-allocates all blocks at boot in DRAM. No malloc/free in the
 * audio path — ever. Tasks grab blocks, fill them, pass them along, then
 * return them to the pool.
 *
 * USAGE:
 *   audio_pool_init();                    // call once at boot
 *
 *   audio_block_t *blk = audio_alloc();   // grab from pool
 *   if (!blk) { ... }                     // pool exhausted (tune POOL_SIZE)
 *   audio_block_clear(blk);               // zero out L and R
 *
 *   // fill blk->L[] and blk->R[] ...
 *
 *   audio_free(blk);                      // return to pool
 */

#ifndef AUDIO_BLOCK_H
#define AUDIO_BLOCK_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

/**
 * Samples per block. This is the fundamental processing granularity.
 * 256 samples at 44100 Hz = ~5.8ms latency per block.
 * Smaller = lower latency but more task switching overhead.
 * Must match the I2S DMA buffer frame count.
 */
#define BLOCK_SAMPLES   256

/**
 * Pool size — total number of pre-allocated blocks.
 * Budget: each block = 256 × 4 bytes × 2 channels = 2048 bytes.
 * 32 blocks = 64KB of DRAM.
 *
 * Need at least: NUM_TRACKS (for track rendering) + 1 (mix accumulator)
 *                + 2 (pipeline headroom) = ~7 minimum.
 * 32 gives comfortable headroom for effects that need scratch blocks.
 */
#define POOL_SIZE       32

/** Bytes per block (for DMA/size calculations) */
#define BLOCK_BYTES     (BLOCK_SAMPLES * 2 * sizeof(int32_t))

/* ------------------------------------------------------------------ */
/* Block type                                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    int32_t L[BLOCK_SAMPLES];    /* left channel, 32-bit for headroom */
    int32_t R[BLOCK_SAMPLES];    /* right channel */
} audio_block_t;

/* ------------------------------------------------------------------ */
/* Pool API                                                            */
/* ------------------------------------------------------------------ */

/**
 * Initialize the block pool. Pre-allocates POOL_SIZE blocks in DRAM.
 * Call once from app_main() before starting the audio task.
 * Returns true on success.
 */
bool audio_pool_init(void);

/**
 * Allocate a block from the pool.
 * Non-blocking with a short timeout (5ms).
 * Returns NULL if pool is exhausted — caller must handle this.
 *
 * NOTE: The returned block is NOT zeroed. Call audio_block_clear()
 * if you need a clean slate, or fill it entirely yourself.
 */
audio_block_t *audio_alloc(void);

/**
 * Return a block to the pool. Safe to call with NULL (no-op).
 */
void audio_free(audio_block_t *blk);

/**
 * How many blocks are currently available in the pool.
 * Useful for debug/monitoring — don't call in the hot path.
 */
int audio_pool_available(void);

/* ------------------------------------------------------------------ */
/* Block utilities                                                     */
/* ------------------------------------------------------------------ */

/** Zero out both channels of a block. */
static inline void audio_block_clear(audio_block_t *blk) {
    memset(blk, 0, sizeof(audio_block_t));
}

/**
 * Accumulate src into dst (dst += src) for mixing.
 * Both blocks must be valid.
 */
static inline void audio_block_accumulate(audio_block_t *dst,
                                          const audio_block_t *src) {
    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        dst->L[i] += src->L[i];
        dst->R[i] += src->R[i];
    }
}

/**
 * Apply a gain to both channels in place.
 * gain = 1.0 is unity, 0.5 is -6dB, etc.
 */
static inline void audio_block_gain(audio_block_t *blk, float gain) {
    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        blk->L[i] = (int32_t)(blk->L[i] * gain);
        blk->R[i] = (int32_t)(blk->R[i] * gain);
    }
}

/**
 * Apply stereo panning. pan = -1.0 (hard left) to 1.0 (hard right).
 * 0.0 = center (no change). Uses constant-power panning.
 */
static inline void audio_block_pan(audio_block_t *blk, float pan) {
    float pan_l = (pan <= 0.0f) ? 1.0f : (1.0f - pan);
    float pan_r = (pan >= 0.0f) ? 1.0f : (1.0f + pan);
    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        blk->L[i] = (int32_t)(blk->L[i] * pan_l);
        blk->R[i] = (int32_t)(blk->R[i] * pan_r);
    }
}

/**
 * Convert int32 block to interleaved int16 stereo for I2S DMA.
 * Applies hard clipping. This is the LAST step before I2S write.
 *
 * @param blk       Source block (int32 L/R)
 * @param out       Destination buffer, must hold BLOCK_SAMPLES * 2 int16s
 * @param divisor   Number of tracks mixed (prevents clipping: sample / divisor)
 */
static inline void audio_block_to_i2s(const audio_block_t *blk,
                                       int16_t *out, int divisor) {
    if (divisor < 1) divisor = 1;
    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        int32_t L = blk->L[i] / divisor;
        int32_t R = blk->R[i] / divisor;

        /* Hard clip */
        if (L >  32767) L =  32767;
        if (L < -32768) L = -32768;
        if (R >  32767) R =  32767;
        if (R < -32768) R = -32768;

        out[i * 2]     = (int16_t)L;
        out[i * 2 + 1] = (int16_t)R;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_BLOCK_H */
