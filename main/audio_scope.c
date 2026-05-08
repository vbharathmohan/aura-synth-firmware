#include "audio_scope.h"

#include <string.h>

static int16_t s_ring[AUDIO_SCOPE_SAMPLES];
static volatile uint32_t s_widx;
static bool s_inited;

void audio_scope_init(void)
{
    if (s_inited) {
        return;
    }
    memset(s_ring, 0, sizeof(s_ring));
    s_widx = 0;
    s_inited = true;
}

void audio_scope_push_i32(const int32_t *samples, int count)
{
    if (!s_inited || samples == NULL || count <= 0) {
        return;
    }
    /* Downscale int32 mix bus to int16 for plotting. */
    for (int i = 0; i < count; i++) {
        int32_t x = samples[i];
        if (x > 32767) x = 32767;
        if (x < -32768) x = -32768;
        uint32_t w = s_widx++;
        s_ring[w % AUDIO_SCOPE_SAMPLES] = (int16_t)x;
    }
}

int audio_scope_read_i16(int16_t *out, int max_count)
{
    if (!s_inited || out == NULL || max_count <= 0) {
        return 0;
    }
    if (max_count > AUDIO_SCOPE_SAMPLES) {
        max_count = AUDIO_SCOPE_SAMPLES;
    }
    /* Snapshot write index and read the last `max_count` samples. */
    uint32_t w = s_widx;
    uint32_t start = (w >= (uint32_t)max_count) ? (w - (uint32_t)max_count) : 0;
    for (int i = 0; i < max_count; i++) {
        out[i] = s_ring[(start + (uint32_t)i) % AUDIO_SCOPE_SAMPLES];
    }
    return max_count;
}

