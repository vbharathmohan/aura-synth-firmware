/**
 * effects.c — Audio effects implementations.
 *
 * Biquad filter uses the Audio EQ Cookbook formula for LPF.
 * Delay allocates its ring buffer in PSRAM (4MB available).
 */

#include "effects.h"
#include <math.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "effects";

/* ================================================================== */
/* Volume                                                              */
/* ================================================================== */

void fx_volume(audio_block_t *block, void *params)
{
    float gain = *(float *)params;
    for (int i = 0; i < BLOCK_SAMPLES; i++)
    {
        block->L[i] = (int32_t)(block->L[i] * gain);
        block->R[i] = (int32_t)(block->R[i] * gain);
    }
}

/* ================================================================== */
/* Biquad filter                                                       */
/* ================================================================== */

static void biquad_calc_coeffs(biquad_t *f, float fc, float q, float fs)
{
    /* Audio EQ Cookbook — LPF */
    float w0 = 2.0f * M_PI * fc / fs;
    float alpha = sinf(w0) / (2.0f * q);
    float cos_w0 = cosf(w0);

    float a0 = 1.0f + alpha;
    f->b0 = ((1.0f - cos_w0) / 2.0f) / a0;
    f->b1 = (1.0f - cos_w0) / a0;
    f->b2 = ((1.0f - cos_w0) / 2.0f) / a0;
    f->a1 = (-2.0f * cos_w0) / a0;
    f->a2 = (1.0f - alpha) / a0;
}

void biquad_init_lpf(biquad_t *f, float cutoff_hz, float q, float sample_rate)
{
    memset(f, 0, sizeof(biquad_t));
    biquad_calc_coeffs(f, cutoff_hz, q, sample_rate);
}

void biquad_update_cutoff(biquad_t *f, float cutoff_hz, float q, float sample_rate)
{
    /* Recalculate coefficients without resetting state */
    biquad_calc_coeffs(f, cutoff_hz, q, sample_rate);
}

void fx_biquad(audio_block_t *block, void *params)
{
    biquad_t *f = (biquad_t *)params;

    for (int i = 0; i < BLOCK_SAMPLES; i++)
    {
        /* Left channel */
        float xL = (float)block->L[i];
        float yL = f->b0 * xL + f->b1 * f->x1L + f->b2 * f->x2L - f->a1 * f->y1L - f->a2 * f->y2L;
        f->x2L = f->x1L;
        f->x1L = xL;
        f->y2L = f->y1L;
        f->y1L = yL;
        block->L[i] = (int32_t)yL;

        /* Right channel */
        float xR = (float)block->R[i];
        float yR = f->b0 * xR + f->b1 * f->x1R + f->b2 * f->x2R - f->a1 * f->y1R - f->a2 * f->y2R;
        f->x2R = f->x1R;
        f->x1R = xR;
        f->y2R = f->y1R;
        f->y1R = yR;
        block->R[i] = (int32_t)yR;
    }
}

/* ================================================================== */
/* Delay                                                               */
/* ================================================================== */

bool delay_init(delay_t *d, float delay_ms, float feedback,
                float mix, float sample_rate)
{
    memset(d, 0, sizeof(delay_t));

    d->buf_len = (size_t)(delay_ms / 1000.0f * sample_rate);
    d->feedback = feedback;
    d->mix = mix;

    if (d->buf_len == 0)
    {
        ESP_LOGW(TAG, "Delay time too short, setting to 1 sample");
        d->buf_len = 1;
    }

    size_t buf_bytes = d->buf_len * sizeof(int32_t);

    /* Allocate in PSRAM */
    d->buf_L = (int32_t *)heap_caps_calloc(d->buf_len, sizeof(int32_t),
                                           MALLOC_CAP_SPIRAM);
    d->buf_R = (int32_t *)heap_caps_calloc(d->buf_len, sizeof(int32_t),
                                           MALLOC_CAP_SPIRAM);

    if (d->buf_L == NULL || d->buf_R == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate delay buffer (%u KB) in PSRAM",
                 (unsigned)(buf_bytes * 2 / 1024));
        delay_deinit(d);
        return false;
    }

    d->write_pos = 0;
    d->initialized = true;

    ESP_LOGI(TAG, "Delay initialized: %.0fms, %u samples, %.0f KB PSRAM",
             delay_ms, (unsigned)d->buf_len, (float)(buf_bytes * 2) / 1024.0f);
    return true;
}

void delay_deinit(delay_t *d)
{
    if (d->buf_L)
    {
        free(d->buf_L);
        d->buf_L = NULL;
    }
    if (d->buf_R)
    {
        free(d->buf_R);
        d->buf_R = NULL;
    }
    d->initialized = false;
}

void fx_delay(audio_block_t *block, void *params)
{
    delay_t *d = (delay_t *)params;
    if (!d->initialized)
        return;

    for (int i = 0; i < BLOCK_SAMPLES; i++)
    {
        /* Read delayed sample */
        int32_t delayed_L = d->buf_L[d->write_pos];
        int32_t delayed_R = d->buf_R[d->write_pos];

        /* Write current + feedback into buffer */
        d->buf_L[d->write_pos] = block->L[i] + (int32_t)(delayed_L * d->feedback);
        d->buf_R[d->write_pos] = block->R[i] + (int32_t)(delayed_R * d->feedback);

        /* Mix delayed signal into output */
        block->L[i] += (int32_t)(delayed_L * d->mix);
        block->R[i] += (int32_t)(delayed_R * d->mix);

        /* Advance write position */
        d->write_pos = (d->write_pos + 1) % d->buf_len;
    }
}
