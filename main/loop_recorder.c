/**
 * loop_recorder.c — Gesture tape recorder.
 *
 * State machine:
 *   IDLE → (record pressed) → RECORDING
 *   RECORDING → (record pressed) → PLAYING (loop length set)
 *   PLAYING → (record pressed on different track) → OVERDUBBING
 *   any state → (clear pressed) → clears active track tape
 *
 * Each track's tape is an array of control_frame_t in PSRAM.
 * At 100 Hz control rate, MAX_LOOP_FRAMES=2000 gives 20 seconds of loop.
 * Memory: 4 tracks × 2000 frames × 8 bytes = 64 KB PSRAM.
 */

#include "loop_recorder.h"
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "loop_rec";

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */

typedef enum {
    REC_IDLE,
    REC_RECORDING,
    REC_PLAYING,
} rec_state_t;

static rec_state_t    s_state = REC_IDLE;
static control_frame_t *s_tapes[NUM_TRACKS];  /* PSRAM-allocated */
static bool            s_tape_has_data[NUM_TRACKS];
static int             s_loop_length = 0;     /* in frames */
static int             s_playhead = 0;
static int             s_rec_track = 0;       /* which track we're recording */

/* ------------------------------------------------------------------ */
/* Init / deinit                                                       */
/* ------------------------------------------------------------------ */

bool loop_recorder_init(void)
{
    memset(s_tapes, 0, sizeof(s_tapes));
    memset(s_tape_has_data, 0, sizeof(s_tape_has_data));
    s_state = REC_IDLE;
    s_loop_length = 0;
    s_playhead = 0;

    for (int i = 0; i < NUM_TRACKS; i++) {
        s_tapes[i] = (control_frame_t *)heap_caps_calloc(
            MAX_LOOP_FRAMES, sizeof(control_frame_t), MALLOC_CAP_SPIRAM);

        if (s_tapes[i] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate tape %d in PSRAM", i);
            loop_recorder_deinit();
            return false;
        }
    }

    size_t total = NUM_TRACKS * MAX_LOOP_FRAMES * sizeof(control_frame_t);
    ESP_LOGI(TAG, "Loop recorder initialized: %d tracks × %d frames = %u KB PSRAM",
             NUM_TRACKS, MAX_LOOP_FRAMES, (unsigned)(total / 1024));
    return true;
}

void loop_recorder_deinit(void)
{
    for (int i = 0; i < NUM_TRACKS; i++) {
        if (s_tapes[i]) {
            free(s_tapes[i]);
            s_tapes[i] = NULL;
        }
    }
    s_state = REC_IDLE;
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/** Capture the current track params into a compact control frame. */
static control_frame_t capture_frame(const shared_state_t *snap)
{
    const track_params_t *tp = &snap->tracks[snap->active_track];
    control_frame_t f;
    f.pitch         = tp->pitch;
    f.volume        = (uint8_t)(tp->volume * 255.0f);
    f.lfo_rate      = (uint8_t)((tp->lfo_rate / 20.0f) * 255.0f);
    f.waveform_mix  = (uint8_t)(tp->waveform_mix * 255.0f);
    f.detune        = (uint8_t)((tp->detune / 10.0f) * 255.0f);
    f.filter_cutoff = (uint8_t)(((tp->filter_cutoff - 100.0f) / 1900.0f) * 255.0f);
    f.pitch_bend    = (uint8_t)((tp->pitch_bend + 1.0f) * 127.5f);
    f.drum_trigger_flags = 0;  /* drums recorded separately if needed */
    return f;
}

/** Apply a recorded control frame back to a track's params. */
static void apply_frame(const control_frame_t *f, track_params_t *tp)
{
    tp->pitch         = f->pitch;
    tp->volume        = f->volume / 255.0f;
    tp->lfo_rate      = (f->lfo_rate / 255.0f) * 20.0f;
    tp->waveform_mix  = f->waveform_mix / 255.0f;
    tp->detune        = (f->detune / 255.0f) * 10.0f;
    tp->filter_cutoff = 100.0f + (f->filter_cutoff / 255.0f) * 1900.0f;
    tp->pitch_bend    = (f->pitch_bend / 127.5f) - 1.0f;
}

/* ------------------------------------------------------------------ */
/* Update (called each control cycle)                                  */
/* ------------------------------------------------------------------ */

void loop_recorder_update(shared_state_t *snap)
{
    /* --- Handle CLEAR --- */
    if (snap->clear_pressed) {
        loop_recorder_clear_track(snap->active_track);
        ESP_LOGI(TAG, "Cleared track %d", snap->active_track);

        /* If no tracks have data, reset to idle */
        if (!loop_recorder_has_data()) {
            s_state = REC_IDLE;
            s_loop_length = 0;
            s_playhead = 0;
        }
    }

    /* --- Handle RECORD toggle --- */
    if (snap->record_pressed) {
        switch (s_state) {
            case REC_IDLE:
                /* Start recording on active track */
                s_rec_track = snap->active_track;
                s_playhead = 0;
                s_loop_length = 0;
                s_state = REC_RECORDING;
                ESP_LOGI(TAG, "Recording started on track %d", s_rec_track);
                break;

            case REC_RECORDING:
                /* Stop recording, set loop length, start playback */
                if (s_playhead > 0) {
                    s_loop_length = s_playhead;
                    s_tape_has_data[s_rec_track] = true;
                    s_playhead = 0;
                    s_state = REC_PLAYING;
                    ESP_LOGI(TAG, "Recording stopped: %d frames (%.1fs). Playing.",
                             s_loop_length, s_loop_length / 100.0f);
                } else {
                    /* Nothing recorded, go back to idle */
                    s_state = REC_IDLE;
                }
                break;

            case REC_PLAYING:
                /* Start overdubbing on the current active track */
                s_rec_track = snap->active_track;
                s_state = REC_RECORDING;
                ESP_LOGI(TAG, "Overdub started on track %d", s_rec_track);
                break;
        }
    }

    /* --- Record current frame --- */
    if (s_state == REC_RECORDING) {
        if (s_playhead < MAX_LOOP_FRAMES) {
            s_tapes[s_rec_track][s_playhead] = capture_frame(snap);
            s_playhead++;
        } else {
            /* Hit max length — auto-stop */
            s_loop_length = MAX_LOOP_FRAMES;
            s_tape_has_data[s_rec_track] = true;
            s_playhead = 0;
            s_state = REC_PLAYING;
            ESP_LOGW(TAG, "Max loop length reached, auto-stopped");
        }
    }

    /* --- Playback recorded frames --- */
    if (s_state == REC_PLAYING || (s_state == REC_RECORDING && s_loop_length > 0)) {
        for (int t = 0; t < NUM_TRACKS; t++) {
            /* Don't play back the track currently being recorded */
            if (s_state == REC_RECORDING && t == s_rec_track) continue;

            if (s_tape_has_data[t] && s_playhead < s_loop_length) {
                apply_frame(&s_tapes[t][s_playhead], &snap->tracks[t]);
            }
        }
    }

    /* --- Advance playhead --- */
    if (s_state == REC_PLAYING) {
        s_playhead++;
        if (s_playhead >= s_loop_length) {
            s_playhead = 0;  /* loop back */
        }
    }

    /* --- Write transport state back to shared state --- */
    snap->is_recording = (s_state == REC_RECORDING);
    snap->is_playing   = (s_state == REC_PLAYING);
    snap->loop_length  = s_loop_length;
    snap->playhead     = s_playhead;
}

/* ------------------------------------------------------------------ */
/* Queries                                                             */
/* ------------------------------------------------------------------ */

int loop_recorder_get_length(void)
{
    return s_loop_length;
}

bool loop_recorder_has_data(void)
{
    for (int i = 0; i < NUM_TRACKS; i++) {
        if (s_tape_has_data[i]) return true;
    }
    return false;
}

void loop_recorder_clear_track(int track)
{
    if (track < 0 || track >= NUM_TRACKS) return;
    if (s_tapes[track]) {
        memset(s_tapes[track], 0, MAX_LOOP_FRAMES * sizeof(control_frame_t));
    }
    s_tape_has_data[track] = false;
}

void loop_recorder_clear_all(void)
{
    for (int i = 0; i < NUM_TRACKS; i++) {
        loop_recorder_clear_track(i);
    }
    s_state = REC_IDLE;
    s_loop_length = 0;
    s_playhead = 0;
}
