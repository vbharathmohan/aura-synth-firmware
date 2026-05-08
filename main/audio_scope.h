#ifndef AUDIO_SCOPE_H
#define AUDIO_SCOPE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** A lock-free-ish scope buffer for GUI rendering (single producer, single consumer). */

#define AUDIO_SCOPE_SAMPLES 1024

void audio_scope_init(void);

/** Push a block of int32 samples (mono or left channel). */
void audio_scope_push_i32(const int32_t *samples, int count);

/**
 * Copy the latest waveform into `out` (int16). Returns number of samples copied.
 * Thread-safe for 1 producer + 1 consumer; may return a slightly torn window,
 * which is fine for visualization.
 */
int audio_scope_read_i16(int16_t *out, int max_count);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_SCOPE_H */

