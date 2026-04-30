/**
 * loop_recorder.c — Gesture-event looper.
 *
 * Records note events (slot/velocity/speed/loop) with frame timestamps
 * and replays them by posting the same events to g_note_queue.
 *
 * Frame clock is driven by loop_recorder_update() at control rate
 * (~86-100 Hz depending on CONTROL_DIVIDER). All timing is quantized
 * to that grid, which is fine for this class project and keeps the
 * implementation deterministic and low overhead.
 */

#include "loop_recorder.h"

#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "loop_rec";

/* Worst-case event budget per track. */
#define MAX_EVENTS_PER_TRACK   4096

typedef struct {
    uint16_t frame;     /* 0..MAX_LOOP_FRAMES-1 */
    note_event_t evt;
} rec_event_t;

static rec_event_t *s_tracks[NUM_TRACKS];
static int          s_event_count[NUM_TRACKS];

static bool s_recording = false;
static bool s_playing = false;
static int  s_record_track = 0;
static int  s_record_frame = 0;      /* used when loop length not yet fixed */

static int  s_loop_length = 0;        /* frames; set by first completed take */
static int  s_playhead = 0;           /* [0, s_loop_length) while playing */

static bool all_tracks_empty(void)
{
    for (int t = 0; t < NUM_TRACKS; t++) {
        if (s_event_count[t] > 0) return false;
    }
    return true;
}

static void clear_track_internal(int track)
{
    if (track < 0 || track >= NUM_TRACKS) return;
    s_event_count[track] = 0;
}

static void start_recording(int track)
{
    if (track < 0 || track >= NUM_TRACKS) track = 0;
    s_recording = true;
    s_record_track = track;
    clear_track_internal(track); /* overwrite policy */

    if (s_loop_length == 0) {
        s_record_frame = 0;
        s_playhead = 0;
    } else {
        /* Record into existing timeline from current playhead. */
        if (s_playhead >= s_loop_length) s_playhead = 0;
    }

    ESP_LOGI(TAG, "REC start track=%d loop_len=%d", s_record_track, s_loop_length);
}

static void stop_recording(void)
{
    if (!s_recording) return;
    s_recording = false;

    if (s_loop_length == 0) {
        /* First take determines loop length. */
        if (s_record_frame <= 0) {
            s_loop_length = 0;
            s_playing = false;
            ESP_LOGI(TAG, "REC stop: empty take");
            return;
        }

        s_loop_length = s_record_frame;
        if (s_loop_length > MAX_LOOP_FRAMES) s_loop_length = MAX_LOOP_FRAMES;
        s_playhead = 0;
        s_playing = true;
        ESP_LOGI(TAG, "REC stop: first loop set (%d frames, %.2fs @100Hz)",
                 s_loop_length, s_loop_length / 100.0f);
    } else {
        /* Existing loop stays fixed-length. */
        s_playing = true;
        ESP_LOGI(TAG, "REC stop: overdub track=%d (loop %d frames)",
                 s_record_track, s_loop_length);
    }
}

bool loop_recorder_init(void)
{
    memset(s_tracks, 0, sizeof(s_tracks));
    memset(s_event_count, 0, sizeof(s_event_count));

    s_recording = false;
    s_playing = false;
    s_record_track = 0;
    s_record_frame = 0;
    s_loop_length = 0;
    s_playhead = 0;

    for (int t = 0; t < NUM_TRACKS; t++) {
        s_tracks[t] = (rec_event_t *)heap_caps_calloc(
            MAX_EVENTS_PER_TRACK, sizeof(rec_event_t), MALLOC_CAP_SPIRAM);
        if (s_tracks[t] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate tape for track %d", t);
            loop_recorder_deinit();
            return false;
        }
    }

    size_t total = NUM_TRACKS * MAX_EVENTS_PER_TRACK * sizeof(rec_event_t);
    ESP_LOGI(TAG, "Init: %d tracks × %d events = %u KB PSRAM",
             NUM_TRACKS, MAX_EVENTS_PER_TRACK, (unsigned)(total / 1024));
    return true;
}

void loop_recorder_deinit(void)
{
    for (int t = 0; t < NUM_TRACKS; t++) {
        if (s_tracks[t]) {
            free(s_tracks[t]);
            s_tracks[t] = NULL;
        }
        s_event_count[t] = 0;
    }
    s_recording = false;
    s_playing = false;
    s_loop_length = 0;
    s_playhead = 0;
}

void loop_recorder_on_live_event(const note_event_t *evt)
{
    if (!s_recording || evt == NULL) return;

    /* Never re-record playback-originated events. */
    if (evt->source == 20) return;

    if (s_record_track < 0 || s_record_track >= NUM_TRACKS) return;
    if (s_tracks[s_record_track] == NULL) return;

    int count = s_event_count[s_record_track];
    if (count >= MAX_EVENTS_PER_TRACK) return; /* drop if full */

    int frame = (s_loop_length > 0) ? s_playhead : s_record_frame;
    if (frame < 0) frame = 0;
    if (frame >= MAX_LOOP_FRAMES) frame = MAX_LOOP_FRAMES - 1;

    s_tracks[s_record_track][count].frame = (uint16_t)frame;
    s_tracks[s_record_track][count].evt = *evt;
    s_event_count[s_record_track] = count + 1;
}

void loop_recorder_update(shared_state_t *snap)
{
    if (snap == NULL) return;

    /* CLEAR active track */
    if (snap->clear_pressed) {
        loop_recorder_clear_track(snap->active_track);
        ESP_LOGI(TAG, "Track %d cleared", snap->active_track);
    }

    /* PLAY / PAUSE */
    if (snap->play_pause_pressed) {
        if (s_loop_length > 0 && loop_recorder_has_data()) {
            s_playing = !s_playing;
            ESP_LOGI(TAG, "Playback %s", s_playing ? "ON" : "PAUSE");
        }
    }

    /* RECORD toggle */
    if (snap->record_pressed) {
        if (!s_recording) {
            start_recording(snap->active_track);
        } else {
            stop_recording();
        }
    }

    /* Playback emit at current playhead before advancing it. */
    if (s_playing && s_loop_length > 0) {
        for (int t = 0; t < NUM_TRACKS; t++) {
            rec_event_t *tape = s_tracks[t];
            int count = s_event_count[t];
            if (tape == NULL || count <= 0) continue;

            for (int i = 0; i < count; i++) {
                if ((int)tape[i].frame == s_playhead) {
                    note_event_t e = tape[i].evt;
                    e.source = 20; /* loop playback provenance */
                    note_event_post(e.slot, e.velocity, e.speed, e.loop, e.source);
                }
            }
        }
    }

    /* Advance clocks */
    if (s_recording) {
        if (s_loop_length == 0) {
            s_record_frame++;
            if (s_record_frame >= MAX_LOOP_FRAMES) {
                /* Auto-stop at max length, sets first loop length. */
                stop_recording();
            }
        }
    }

    if (s_playing && s_loop_length > 0) {
        s_playhead++;
        if (s_playhead >= s_loop_length) s_playhead = 0;
    }

    /* Publish transport state */
    snap->is_recording = s_recording;
    snap->is_playing = s_playing;
    snap->loop_length = s_loop_length;
    snap->playhead = s_playhead;
}

int loop_recorder_get_length(void)
{
    return s_loop_length;
}

bool loop_recorder_has_data(void)
{
    return !all_tracks_empty();
}

void loop_recorder_clear_track(int track)
{
    if (track < 0 || track >= NUM_TRACKS) return;
    clear_track_internal(track);

    if (all_tracks_empty()) {
        s_recording = false;
        s_playing = false;
        s_record_track = 0;
        s_record_frame = 0;
        s_loop_length = 0;
        s_playhead = 0;
        ESP_LOGI(TAG, "All tracks empty -> loop length reset");
    }
}

void loop_recorder_clear_all(void)
{
    for (int t = 0; t < NUM_TRACKS; t++) {
        clear_track_internal(t);
    }
    s_recording = false;
    s_playing = false;
    s_record_track = 0;
    s_record_frame = 0;
    s_loop_length = 0;
    s_playhead = 0;
}
