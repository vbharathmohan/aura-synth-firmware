/**
 * synth_voice.c — Monophonic synthesizer voice.
 *
 * DSP chain per sample:
 *   1. Generate sine wave at base frequency
 *   2. Generate detuned sawtooth at (base freq + detune_hz)
 *   3. Crossfade between sine and saw via waveform_mix
 *   4. Apply LFO tremolo (amplitude modulation)
 *   5. Apply volume
 *   6. Apply one-pole low-pass filter
 *
 * Based on the audio teammate's SynthesizerVoice, adapted for
 * block-based processing and the shared_state parameter interface.
 */

#include "synth_voice.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/** MIDI note → frequency. Note 69 = A4 = 440 Hz. */
static float midi_to_freq(uint8_t note)
{
    return 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
}

/** Naive sawtooth: phase 0..1 → output -1..1 */
static inline float saw(float phase)
{
    return 2.0f * phase - 1.0f;
}

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

void synth_voice_init(synth_voice_t *v, float rate)
{
    v->sample_rate   = rate;
    v->phase_main    = 0.0f;
    v->phase_detune  = 0.0f;
    v->frequency     = 261.63f;  /* C4 */
    v->waveform_mix  = 0.5f;
    v->detune_hz     = 0.0f;
    v->pitch_bend    = 0.0f;
    v->volume        = 0.8f;
    v->lfo_phase     = 0.0f;
    v->lfo_rate      = 5.0f;
    v->filter_cutoff = 2000.0f;
    v->filter_y_L    = 0.0f;
    v->filter_y_R    = 0.0f;
    v->active        = false;
}

/* ------------------------------------------------------------------ */
/* Parameter update                                                    */
/* ------------------------------------------------------------------ */

void synth_voice_set_pitch(synth_voice_t *v, uint8_t midi_note)
{
    v->frequency = midi_to_freq(midi_note);
}

void synth_voice_set_params(synth_voice_t *v, const track_params_t *p)
{
    v->volume       = p->volume;
    v->waveform_mix = p->waveform_mix;
    v->detune_hz    = p->detune;
    v->lfo_rate     = p->lfo_rate;
    v->filter_cutoff = p->filter_cutoff;
    v->pitch_bend   = p->pitch_bend;

    /* Update base frequency from pitch + bend */
    float bent_note = (float)p->pitch + v->pitch_bend * 2.0f;
    v->frequency = 440.0f * powf(2.0f, (bent_note - 69.0f) / 12.0f);
}

/* ------------------------------------------------------------------ */
/* Render one block                                                    */
/* ------------------------------------------------------------------ */

void synth_voice_render(synth_voice_t *v, audio_block_t *blk)
{
    if (!v->active || v->volume < 0.001f) {
        /* Silent — leave block as-is (caller should clear if needed) */
        return;
    }

    const float sr = v->sample_rate;
    const float freq_main   = v->frequency;
    const float freq_detune = v->frequency + v->detune_hz;
    const float mix = v->waveform_mix;
    const float vol = v->volume;

    /* Phase increments (normalized 0..1 per sample) */
    const float inc_main   = freq_main / sr;
    const float inc_detune = freq_detune / sr;
    const float inc_lfo    = v->lfo_rate / sr;

    /* One-pole LP filter coefficient:
     * alpha = dt / (RC + dt), where RC = 1/(2*pi*fc)
     * Simplified: alpha = 2*pi*fc / (2*pi*fc + sr)  */
    float fc = v->filter_cutoff;
    if (fc < 20.0f) fc = 20.0f;
    if (fc > sr * 0.49f) fc = sr * 0.49f;
    const float alpha = (2.0f * M_PI * fc) / (2.0f * M_PI * fc + sr);

    float phase_m = v->phase_main;
    float phase_d = v->phase_detune;
    float phase_l = v->lfo_phase;
    float filt_yL = v->filter_y_L;
    float filt_yR = v->filter_y_R;

    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        /* --- Oscillators --- */
        float sine_val = sinf(phase_m * 2.0f * M_PI);
        float saw_val  = saw(phase_d);

        /* Crossfade: mix=0 → pure sine, mix=1 → pure saw */
        float osc = sine_val * (1.0f - mix) + saw_val * mix;

        /* --- LFO tremolo --- */
        float lfo = 1.0f - 0.3f * (0.5f + 0.5f * sinf(phase_l * 2.0f * M_PI));
        /* lfo ranges from 0.7 to 1.0 — gentle tremolo */

        /* --- Amplitude --- */
        float sample = osc * vol * lfo * 32000.0f;

        /* --- One-pole LP filter (applied identically to L and R) --- */
        filt_yL = filt_yL + alpha * (sample - filt_yL);
        filt_yR = filt_yL;  /* mono source → same filter state */

        int32_t out = (int32_t)filt_yL;

        blk->L[i] = out;
        blk->R[i] = out;

        /* Advance phases (wrap 0..1) */
        phase_m += inc_main;
        if (phase_m >= 1.0f) phase_m -= 1.0f;

        phase_d += inc_detune;
        if (phase_d >= 1.0f) phase_d -= 1.0f;

        phase_l += inc_lfo;
        if (phase_l >= 1.0f) phase_l -= 1.0f;
    }

    /* Store state for next block */
    v->phase_main   = phase_m;
    v->phase_detune = phase_d;
    v->lfo_phase    = phase_l;
    v->filter_y_L   = filt_yL;
    v->filter_y_R   = filt_yR;
}
