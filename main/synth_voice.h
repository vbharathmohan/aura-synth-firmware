/**
 * synth_voice.h — Synthesizer voice for Aura-Synth.
 *
 * One voice = one monophonic synth with:
 *   - Sine + detuned sawtooth oscillator (crossfadeable via waveform_mix)
 *   - One-pole low-pass filter
 *   - LFO tremolo
 *   - Pitch bend (±2 semitones)
 *
 * Fills audio_block_t in the block pipeline. One voice per track.
 *
 * USAGE:
 *   synth_voice_t voice;
 *   synth_voice_init(&voice, SAMPLE_RATE);
 *   synth_voice_set_pitch(&voice, 60);
 *   synth_voice_set_params(&voice, &track_params);
 *
 *   audio_block_t *blk = audio_alloc();
 *   audio_block_clear(blk);
 *   synth_voice_render(&voice, blk);
 */

#ifndef SYNTH_VOICE_H
#define SYNTH_VOICE_H

#include <stdint.h>
#include <stdbool.h>
#include "audio_block.h"
#include "shared_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float sample_rate;

    /* Oscillator */
    float phase_main;
    float phase_detune;
    float frequency;         /* base freq in Hz (from MIDI note) */

    /* Params (updated per block from shared_state) */
    float waveform_mix;      /* 0=sine, 1=saw */
    float detune_hz;
    float pitch_bend;        /* -1..1 → ±2 semitones */
    float volume;

    /* LFO */
    float lfo_phase;
    float lfo_rate;          /* Hz */

    /* One-pole LP filter */
    float filter_cutoff;     /* Hz */
    float filter_y_L;
    float filter_y_R;

    bool  active;
} synth_voice_t;

void synth_voice_init(synth_voice_t *v, float rate);
void synth_voice_set_pitch(synth_voice_t *v, uint8_t midi_note);
void synth_voice_set_params(synth_voice_t *v, const track_params_t *p);
void synth_voice_render(synth_voice_t *v, audio_block_t *blk);

static inline void synth_voice_set_active(synth_voice_t *v, bool on) {
    v->active = on;
}

#ifdef __cplusplus
}
#endif

#endif /* SYNTH_VOICE_H */
