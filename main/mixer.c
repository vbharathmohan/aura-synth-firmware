/**
 * mixer.c — Multi-track mixer with effects chains.
 *
 * The mixer owns:
 *   - One synth_voice_t per track
 *   - One biquad_t per track (LP filter from shared_state cutoff)
 *   - Per-track and master effect chains
 *
 * The sampler is external — mixer_process() calls sampler_render()
 * to accumulate triggered samples into the mix.
 */

#include "mixer.h"
#include "sampler.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "mixer";

/* ------------------------------------------------------------------ */
/* Static state                                                        */
/* ------------------------------------------------------------------ */

static synth_voice_t s_voices[NUM_TRACKS];
static biquad_t      s_track_filters[NUM_TRACKS];
static float         s_sample_rate = 44100.0f;

/* Per-track effects chain */
static effect_fn s_track_fx[NUM_TRACKS][MAX_TRACK_FX];
static void     *s_track_fx_params[NUM_TRACKS][MAX_TRACK_FX];

/* Master effects chain */
static effect_fn s_master_fx[MAX_MASTER_FX];
static void     *s_master_fx_params[MAX_MASTER_FX];

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

bool mixer_init(float sample_rate)
{
    s_sample_rate = sample_rate;

    /* Init synth voices */
    for (int i = 0; i < NUM_TRACKS; i++) {
        synth_voice_init(&s_voices[i], sample_rate);
    }

    /* Init per-track LP filters (wide open by default) */
    for (int i = 0; i < NUM_TRACKS; i++) {
        biquad_init_lpf(&s_track_filters[i], 2000.0f, 0.707f, sample_rate);
    }

    /* Clear effects chains */
    memset(s_track_fx, 0, sizeof(s_track_fx));
    memset(s_track_fx_params, 0, sizeof(s_track_fx_params));
    memset(s_master_fx, 0, sizeof(s_master_fx));
    memset(s_master_fx_params, 0, sizeof(s_master_fx_params));

    /* Wire up the default per-track effect: biquad LP filter in slot 0 */
    for (int i = 0; i < NUM_TRACKS; i++) {
        s_track_fx[i][0] = fx_biquad;
        s_track_fx_params[i][0] = &s_track_filters[i];
    }

    ESP_LOGI(TAG, "Mixer initialized: %d tracks, %.0f Hz", NUM_TRACKS, sample_rate);
    return true;
}

/* ------------------------------------------------------------------ */
/* Effects chain management                                            */
/* ------------------------------------------------------------------ */

void mixer_set_track_fx(int track, int slot, effect_fn fn, void *params)
{
    if (track < 0 || track >= NUM_TRACKS) return;
    if (slot < 0 || slot >= MAX_TRACK_FX) return;
    s_track_fx[track][slot] = fn;
    s_track_fx_params[track][slot] = params;
}

void mixer_set_master_fx(int slot, effect_fn fn, void *params)
{
    if (slot < 0 || slot >= MAX_MASTER_FX) return;
    s_master_fx[slot] = fn;
    s_master_fx_params[slot] = params;
}

/* ------------------------------------------------------------------ */
/* Process one audio cycle                                             */
/* ------------------------------------------------------------------ */

audio_block_t *mixer_process(const shared_state_t *snap)
{
    /* Allocate the mix accumulator */
    audio_block_t *mix = audio_alloc();
    if (mix == NULL) return NULL;
    audio_block_clear(mix);

    /* --- Render each track --- */
    for (int t = 0; t < NUM_TRACKS; t++) {
        const track_params_t *tp = &snap->tracks[t];

        /* Update voice parameters from shared state */
        synth_voice_set_params(&s_voices[t], tp);

        /* Update per-track filter cutoff */
        biquad_update_cutoff(&s_track_filters[t], tp->filter_cutoff,
                             0.707f, s_sample_rate);

        /* Decide if this track's synth voice should be active */
        bool synth_active = (snap->mode == MODE_SYNTH &&
                             t == snap->active_track &&
                             tp->volume > 0.01f);
        synth_voice_set_active(&s_voices[t], synth_active);

        if (!s_voices[t].active) continue;

        /* Allocate a track block */
        audio_block_t *tb = audio_alloc();
        if (tb == NULL) continue;
        audio_block_clear(tb);

        /* Render synth voice into the track block */
        synth_voice_render(&s_voices[t], tb);

        /* Apply per-track effects chain */
        for (int fx = 0; fx < MAX_TRACK_FX; fx++) {
            if (s_track_fx[t][fx] != NULL) {
                s_track_fx[t][fx](tb, s_track_fx_params[t][fx]);
            }
        }

        /* Accumulate into mix bus */
        audio_block_accumulate(mix, tb);
        audio_free(tb);
    }

    /* --- Process drum/piano triggers --- */
    if (snap->drum.trigger) {
        if (snap->mode == MODE_DRUMS) {
            sampler_trigger(snap->drum.slot, snap->drum.velocity, 1.0f, false);
        } else if (snap->mode == MODE_PIANO) {
            /* Piano: pitch-shift a base sample. Slot 8 = piano base.
             * Map piano_note (0-7) to speed via semitone ratio. */
            float speed = powf(2.0f, (float)snap->drum.slot / 12.0f);
            sampler_trigger(8, snap->drum.velocity, speed, false);
        }
    }

    /* Render all active sampler voices into the mix */
    sampler_render(mix);

    /* --- Apply master volume --- */
    audio_block_gain(mix, snap->master_volume);

    /* --- Apply master effects chain --- */
    for (int fx = 0; fx < MAX_MASTER_FX; fx++) {
        if (s_master_fx[fx] != NULL) {
            s_master_fx[fx](mix, s_master_fx_params[fx]);
        }
    }

    return mix;
}

/* ------------------------------------------------------------------ */
/* Accessors                                                           */
/* ------------------------------------------------------------------ */

synth_voice_t *mixer_get_voice(int track)
{
    if (track < 0 || track >= NUM_TRACKS) return NULL;
    return &s_voices[track];
}

biquad_t *mixer_get_track_filter(int track)
{
    if (track < 0 || track >= NUM_TRACKS) return NULL;
    return &s_track_filters[track];
}
