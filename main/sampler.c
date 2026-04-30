/**
 * sampler.c — Flash-embedded WAV sample player.
 *
 * Samples live in flash via CMake's target_add_binary_data().
 * The WAV header (44 bytes) is parsed at registration time to
 * extract sample rate, bit depth, and data length.
 * PCM data is accessed via direct pointer — zero copy.
 *
 * Voice stealing: when all voices are busy, the oldest voice
 * (furthest playback position) is stolen for the new trigger.
 */

#include "sampler.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "sampler";

/* ------------------------------------------------------------------ */
/* WAV parsing helpers                                                  */
/* ------------------------------------------------------------------ */

static inline uint16_t rd_le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rd_le32(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

/* ------------------------------------------------------------------ */
/* Static state                                                        */
/* ------------------------------------------------------------------ */

static sample_slot_t  s_slots[SAMPLER_MAX_SLOTS];
static sampler_voice_t s_voices[SAMPLER_MAX_VOICES];
static int             s_slot_count = 0;

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

void sampler_init(void)
{
    memset(s_slots, 0, sizeof(s_slots));
    memset(s_voices, 0, sizeof(s_voices));
    s_slot_count = 0;
    ESP_LOGI(TAG, "Sampler initialized: %d slots, %d voices",
             SAMPLER_MAX_SLOTS, SAMPLER_MAX_VOICES);
}

/* ------------------------------------------------------------------ */
/* Register                                                            */
/* ------------------------------------------------------------------ */

bool sampler_register(int slot_idx, const char *name,
                      const uint8_t *wav_start, const uint8_t *wav_end)
{
    if (slot_idx < 0 || slot_idx >= SAMPLER_MAX_SLOTS) {
        ESP_LOGE(TAG, "Slot %d out of range", slot_idx);
        return false;
    }

    size_t file_size = wav_end - wav_start;
    if (file_size < 44) {
        ESP_LOGE(TAG, "'%s': file too small (%d bytes)", name, (int)file_size);
        return false;
    }

    /* Validate */
    if (memcmp(wav_start, "RIFF", 4) != 0 ||
        memcmp(wav_start + 8, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "'%s': not a valid WAV file", name);
        return false;
    }

    uint16_t audio_format = 0;
    uint16_t num_channels = 0;
    uint16_t bits_per_sample = 0;
    uint32_t sample_rate = 0;
    const uint8_t *data_ptr = NULL;
    uint32_t data_size = 0;
    bool got_fmt = false;
    bool got_data = false;

    /* Parse RIFF chunks so WAVs with extra metadata chunks still load. */
    size_t off = 12; /* after RIFF + size + WAVE */
    while (off + 8 <= file_size) {
        const uint8_t *chunk = wav_start + off;
        uint32_t chunk_size = rd_le32(chunk + 4);
        size_t payload_off = off + 8;
        size_t next_off = payload_off + chunk_size + (chunk_size & 1u); /* word align */
        if (payload_off > file_size || next_off > file_size) {
            break;
        }

        if (memcmp(chunk, "fmt ", 4) == 0) {
            if (chunk_size < 16) {
                ESP_LOGE(TAG, "'%s': malformed fmt chunk", name);
                return false;
            }
            const uint8_t *fmt = wav_start + payload_off;
            audio_format = rd_le16(fmt + 0);
            num_channels = rd_le16(fmt + 2);
            sample_rate = rd_le32(fmt + 4);
            bits_per_sample = rd_le16(fmt + 14);
            got_fmt = true;
        } else if (memcmp(chunk, "data", 4) == 0) {
            data_ptr = wav_start + payload_off;
            data_size = chunk_size;
            got_data = true;
        }

        off = next_off;
    }

    if (!got_fmt || !got_data || data_ptr == NULL || data_size == 0) {
        ESP_LOGE(TAG, "'%s': missing fmt/data chunks", name);
        return false;
    }

    if (audio_format != 1) {
        ESP_LOGE(TAG, "'%s': not PCM (format=%d)", name, audio_format);
        return false;
    }

    if (num_channels == 0) {
        ESP_LOGE(TAG, "'%s': invalid channel count", name);
        return false;
    }

    if (bits_per_sample != 16) {
        ESP_LOGE(TAG, "'%s': not 16-bit (got %d-bit)", name, bits_per_sample);
        return false;
    }

    const int16_t *pcm = (const int16_t *)data_ptr;
    uint32_t num_samples = data_size / sizeof(int16_t) / num_channels;

    sample_slot_t *slot = &s_slots[slot_idx];
    strncpy(slot->name, name, SAMPLER_NAME_LEN - 1);
    slot->name[SAMPLER_NAME_LEN - 1] = '\0';
    slot->data        = pcm;
    slot->length      = num_samples;
    slot->sample_rate = sample_rate;
    slot->loaded      = true;

    if (slot_idx >= s_slot_count) {
        s_slot_count = slot_idx + 1;
    }

    ESP_LOGI(TAG, "Registered [%d] '%s': %lu samples, %lu Hz, %dch, %.2fs",
             slot_idx, name,
             (unsigned long)num_samples,
             (unsigned long)sample_rate,
             num_channels,
             (float)num_samples / sample_rate);

    return true;
}

/* ------------------------------------------------------------------ */
/* Trigger                                                             */
/* ------------------------------------------------------------------ */

int sampler_trigger(int slot_idx, uint8_t velocity, float speed, bool loop)
{
    if (slot_idx < 0 || slot_idx >= SAMPLER_MAX_SLOTS ||
        !s_slots[slot_idx].loaded) {
        return -1;
    }

    /* Find a free voice, or steal the oldest */
    int free_v = -1;
    float oldest_pos = -1.0f;
    int oldest_v = 0;

    for (int i = 0; i < SAMPLER_MAX_VOICES; i++) {
        if (!s_voices[i].active) {
            free_v = i;
            break;
        }
        if (s_voices[i].position > oldest_pos) {
            oldest_pos = s_voices[i].position;
            oldest_v = i;
        }
    }

    int v = (free_v >= 0) ? free_v : oldest_v;

    s_voices[v].slot     = &s_slots[slot_idx];
    s_voices[v].position = 0.0f;
    s_voices[v].speed    = speed;
    s_voices[v].volume   = velocity / 255.0f;
    s_voices[v].active   = true;
    s_voices[v].looping  = loop;

    ESP_LOGD(TAG, "Trigger [%d] '%s' → voice %d (vel=%d spd=%.2f)",
             slot_idx, s_slots[slot_idx].name, v, velocity, speed);

    return v;
}

/* ------------------------------------------------------------------ */
/* Stop                                                                */
/* ------------------------------------------------------------------ */

void sampler_stop_voice(int voice_idx)
{
    if (voice_idx >= 0 && voice_idx < SAMPLER_MAX_VOICES) {
        s_voices[voice_idx].active = false;
    }
}

void sampler_stop_all(void)
{
    for (int i = 0; i < SAMPLER_MAX_VOICES; i++) {
        s_voices[i].active = false;
    }
}

/* ------------------------------------------------------------------ */
/* Render (ACCUMULATES into block — does not clear)                    */
/* ------------------------------------------------------------------ */

void sampler_render(audio_block_t *blk)
{
    for (int v = 0; v < SAMPLER_MAX_VOICES; v++) {
        sampler_voice_t *voice = &s_voices[v];
        if (!voice->active || !voice->slot) continue;

        const int16_t *data = voice->slot->data;
        const uint32_t len  = voice->slot->length;
        const float vol     = voice->volume;

        for (int i = 0; i < BLOCK_SAMPLES; i++) {
            uint32_t idx = (uint32_t)voice->position;

            if (idx >= len) {
                if (voice->looping) {
                    voice->position = 0.0f;
                    idx = 0;
                } else {
                    voice->active = false;
                    break;
                }
            }

            /* Linear interpolation for smooth pitch shifting */
            float frac = voice->position - (float)idx;
            int32_t s0 = data[idx];
            int32_t s1 = (idx + 1 < len) ? data[idx + 1] : s0;
            int32_t interp = s0 + (int32_t)(frac * (float)(s1 - s0));

            int32_t out = (int32_t)(interp * vol);

            /* Accumulate (mono sample → both channels) */
            blk->L[i] += out;
            blk->R[i] += out;

            voice->position += voice->speed;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Queries                                                             */
/* ------------------------------------------------------------------ */

int sampler_active_count(void)
{
    int count = 0;
    for (int i = 0; i < SAMPLER_MAX_VOICES; i++) {
        if (s_voices[i].active) count++;
    }
    return count;
}

const sample_slot_t *sampler_get_slot(int slot_idx)
{
    if (slot_idx < 0 || slot_idx >= SAMPLER_MAX_SLOTS) return NULL;
    if (!s_slots[slot_idx].loaded) return NULL;
    return &s_slots[slot_idx];
}
