/**
 * shared_state.h — Cross-core communication for Aura-Synth.
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │  THIS FILE IS THE CONTRACT BETWEEN ALL TEAMMATES.       │
 * │  Do NOT change it without agreement from everyone.      │
 * │  Core 0 (sensors) WRITES. Core 1 (audio) READS.        │
 * └─────────────────────────────────────────────────────────┘
 *
 * USAGE (Core 0 — sensor/control task):
 *   shared_state_lock();
 *   g_state.tracks[0].pitch = 60;
 *   g_state.drum_trigger = true;
 *   shared_state_unlock();
 *
 * USAGE (Core 1 — audio task):
 *   shared_state_t snap = {0};
 *   shared_state_snapshot(&snap);
 *   // use snap.tracks[0].pitch etc. — lock is released
 */

#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

#define NUM_TRACKS          4
#define NUM_DRUM_SLOTS      8     /* kick, snare, hihat, etc. */
#define MAX_LOOP_FRAMES     2000  /* 20 seconds at 100 Hz control rate */

/* ------------------------------------------------------------------ */
/* Enums                                                               */
/* ------------------------------------------------------------------ */

typedef enum {
    MODE_SYNTH = 0,
    MODE_DRUMS = 1,
    MODE_PIANO = 2,
    MODE_FX    = 3,
    MODE_COUNT
} instrument_mode_t;

/* ------------------------------------------------------------------ */
/* Per-track synth parameters                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    /* Pitch */
    uint8_t  pitch;              /* MIDI note 36-84 */
    float    pitch_bend;         /* -1.0 to 1.0, 0.0 = center */

    /* Amplitude */
    float    volume;             /* 0.0 - 1.0 */

    /* Oscillator */
    float    waveform_mix;       /* 0.0 = pure sine, 1.0 = pure saw */
    float    detune;             /* 0.0 - 10.0 Hz detuning */

    /* Modulation */
    float    lfo_rate;           /* 0.01 - 20.0 Hz */

    /* Filter */
    float    filter_cutoff;      /* 100.0 - 2000.0 Hz */
} track_params_t;

/* ------------------------------------------------------------------ */
/* Control frame — one snapshot of gesture state for loop recording     */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t pitch;
    uint8_t volume;
    uint8_t lfo_rate;
    uint8_t waveform_mix;
    uint8_t detune;
    uint8_t filter_cutoff;
    uint8_t pitch_bend;
    uint8_t drum_trigger_flags;  /* bit per drum slot: bit 0 = kick, etc. */
} control_frame_t;

/* ------------------------------------------------------------------ */
/* Drum trigger                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t slot;                /* 0-7: which drum sound */
    uint8_t velocity;            /* 0-255 */
    bool    trigger;             /* set by Core 0, consumed by Core 1 */
} drum_trigger_t;

/* ------------------------------------------------------------------ */
/* Main shared state                                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    /* --- Mode & transport --- */
    instrument_mode_t mode;
    uint8_t  active_track;       /* 0 to NUM_TRACKS-1 */
    bool     master_view;        /* true = master bus, false = track view */

    /* --- Per-track synth parameters --- */
    track_params_t tracks[NUM_TRACKS];

    /* --- Master bus --- */
    float    master_volume;      /* 0.0 - 1.0 */
    float    master_filter;      /* 100.0 - 2000.0 Hz */

    /* --- Drum triggers (one-shot, consumed by Core 1) --- */
    drum_trigger_t drum;

    /* --- Loop recorder transport --- */
    bool     record_pressed;     /* edge-detected by Core 0, consumed once */
    bool     clear_pressed;      /* edge-detected by Core 0, consumed once */
    bool     is_recording;       /* current state (managed by loop_recorder) */
    bool     is_playing;         /* current state (managed by loop_recorder) */
    int      loop_length;        /* 0 = no loop set yet */
    int      playhead;           /* current position in the loop */

} shared_state_t;

/* ------------------------------------------------------------------ */
/* Global instance + mutex                                             */
/* ------------------------------------------------------------------ */

extern shared_state_t    g_state;
extern SemaphoreHandle_t g_state_mutex;

/* ------------------------------------------------------------------ */
/* Init / lock / unlock                                                */
/* ------------------------------------------------------------------ */

/** Initialize with sensible defaults. Call once from app_main(). */
void shared_state_init(void);

/** Lock (up to 10ms timeout). Returns true if acquired. */
static inline bool shared_state_lock(void) {
    return xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE;
}

/** Unlock. */
static inline void shared_state_unlock(void) {
    xSemaphoreGive(g_state_mutex);
}

/**
 * Copy the entire state into a local snapshot and consume one-shot triggers.
 * Core 1 calls this once per audio cycle to minimize lock hold time.
 */
static inline void shared_state_snapshot(shared_state_t *out) {
    if (shared_state_lock()) {
        *out = g_state;
        /* Consume one-shot events */
        g_state.drum.trigger = false;
        g_state.record_pressed = false;
        g_state.clear_pressed = false;
        shared_state_unlock();
    }
}

#ifdef __cplusplus
}
#endif

#endif /* SHARED_STATE_H */
