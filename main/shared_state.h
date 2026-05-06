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
#include "freertos/queue.h"
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
/* Sample slot map                                                     */
/* ------------------------------------------------------------------ */
/* These indices are the contract between sensor_task (which posts     */
/* note events) and main.c (which calls sampler_register).             */

#define SAMPLE_SLOT_PIANO       0
#define SAMPLE_SLOT_STEEL_DRUM  1
#define SAMPLE_SLOT_TRUMPET     2
#define SAMPLE_SLOT_808_BASS    3
#define SAMPLE_SLOT_KICK        4
#define SAMPLE_SLOT_SNARE       5
#define SAMPLE_SLOT_HIHAT       6
#define SAMPLE_SLOT_CLAP        7

#define NUM_INSTRUMENTS         4    /* piano, steel drum, trumpet, 808 */
#define NUM_DRUM_PADS           4    /* kick, snare, hihat, clap */
#define NUM_TOF_SENSORS         8    /* C-major scale: C D E F G A B C */

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

/** Which sampled instrument the ToF array currently plays. */
typedef enum {
    INST_PIANO       = 0,
    INST_STEEL_DRUM  = 1,
    INST_TRUMPET     = 2,
    INST_808_BASS    = 3,
    INST_COUNT
} instrument_t;

/** What the 4-button bank does in integration_mode.
 *  PAD_INSTRUMENTS: each button selects which instrument the ToFs play.
 *  PAD_DRUMS:       each button is a drum pad (ToFs still play current
 *                   instrument; drums play in parallel).
 *  A separate "pad-toggle" button switches between the two. */
typedef enum {
    PAD_INSTRUMENTS = 0,
    PAD_DRUMS       = 1,
} pad_mode_t;

/** Map an INST_* value to the corresponding sampler slot index. */
static inline uint8_t instrument_to_slot(instrument_t inst) {
    switch (inst) {
        case INST_PIANO:      return SAMPLE_SLOT_PIANO;
        case INST_STEEL_DRUM: return SAMPLE_SLOT_STEEL_DRUM;
        case INST_TRUMPET:    return SAMPLE_SLOT_TRUMPET;
        case INST_808_BASS:   return SAMPLE_SLOT_808_BASS;
        default:              return SAMPLE_SLOT_PIANO;
    }
}

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

    /* --- ToF instrument selection --- */
    instrument_t active_instrument;  /* what the ToFs currently play */
    pad_mode_t   pad_mode;           /* 4-button bank: instruments vs drums */

    /* --- Per-track synth parameters --- */
    track_params_t tracks[NUM_TRACKS];

    /* --- Master bus --- */
    float    master_volume;      /* 0.0 - 1.0 */
    float    master_filter;      /* 100.0 - 2000.0 Hz */
    float    master_delay_mix;   /* 0.0 = dry, 1.0 = full wet */

    /* --- Drum triggers (one-shot, consumed by Core 1) --- */
    drum_trigger_t drum;

    /* --- Loop recorder transport --- */
    bool     record_pressed;     /* edge-detected by Core 0, consumed once */
    bool     clear_pressed;      /* edge-detected by Core 0, consumed once */
    bool     play_pause_pressed; /* edge-detected by Core 0, consumed once */
    bool     is_recording;       /* current state (managed by loop_recorder) */
    bool     is_playing;         /* current state (managed by loop_recorder) */
    int      loop_length;        /* 0 = no loop set yet */
    int      playhead;           /* current position in the loop */

} shared_state_t;

/* ------------------------------------------------------------------ */
/* Note event queue                                                    */
/* ------------------------------------------------------------------ */
/* Many-producers / one-consumer queue of one-shot sample triggers.    */
/* Producers (sensor_task, button_task, future loop playback) post     */
/* events; the audio task drains them once per block and forwards them */
/* to sampler_trigger().                                               */
/*                                                                     */
/* This decouples gesture detection from the audio render path and is  */
/* ready for the future loop-recorder use case (each event carries     */
/* enough info to be re-queued from a tape).                           */

typedef struct {
    uint8_t  slot;       /* sampler slot index (SAMPLE_SLOT_*) */
    uint8_t  velocity;   /* 0-255, mapped to sampler voice volume */
    float    speed;      /* 1.0 = native pitch, 2.0 = +octave, etc. */
    bool     loop;       /* true = loop, false = one-shot */
    int8_t   source;     /* event provenance: -1 unknown,
                            0..7 = ToF sensor index,
                            10..13 = drum pad index 0..3,
                            20 = loop playback */
} note_event_t;

#define NOTE_QUEUE_DEPTH    32

extern QueueHandle_t g_note_queue;

/** Post a note event. Safe to call from any task / any core.
 *  Returns false if the queue is full (the event is dropped).
 *  Designed so the future loop_recorder can also post here. */
bool note_event_post(uint8_t slot, uint8_t velocity, float speed,
                     bool loop, int8_t source);

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
        g_state.play_pause_pressed = false;
        shared_state_unlock();
    }
}

#ifdef __cplusplus
}
#endif

#endif /* SHARED_STATE_H */
