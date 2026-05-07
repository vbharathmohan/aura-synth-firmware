/**
 * loop_recorder.c — Gesture-event looper.
 *
 * Records note events with microsecond-precise wall-clock timestamps
 * (esp_timer_get_time()) and replays them by posting the same events
 * to g_note_queue when the playback head crosses each event's recorded
 * time.
 *
 * Why µs timestamps and not control-rate frames:
 *   The audio task drains the note queue every audio block (~172 Hz)
 *   but loop_recorder_update() only fires every CONTROL_DIVIDER blocks
 *   (~86 Hz). Sensor strikes can be 22 ms apart yet land in the same
 *   ~12 ms control window. Timestamping at the moment the event is
 *   handed to the recorder gives single-µs precision regardless of how
 *   often update() runs, so playback preserves the rhythmic spacing
 *   that was actually performed.
 */

#include "loop_recorder.h"

#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "loop_rec";

/* Worst-case event budget per track. */
#define MAX_EVENTS_PER_TRACK   4096

/* Hard cap on loop length (used to bound first-take auto-stop). */
#define MAX_LOOP_US            (20 * 1000 * 1000)  /* 20 seconds */

typedef struct {
    uint32_t t_us;          /* relative time in µs, 0 .. s_loop_duration_us */
    note_event_t evt;
} rec_event_t;

static rec_event_t *s_tracks[NUM_TRACKS];
static int          s_event_count[NUM_TRACKS];

static bool     s_recording = false;
static bool     s_playing   = false;
static int      s_record_track = 0;

/* Wall-clock anchor: the absolute esp_timer time that corresponds to
 * loop position 0. While playing, current loop position is
 *   (esp_timer_get_time() - s_play_origin_us) % s_loop_duration_us.
 * While paused, s_pause_loop_pos_us holds the position to resume from. */
static int64_t  s_record_start_us = 0;
static int64_t  s_play_origin_us  = 0;
static uint32_t s_pause_loop_pos_us = 0;

static uint32_t s_loop_duration_us = 0;   /* 0 = no loop yet */
static uint32_t s_last_loop_pos_us = 0;   /* set by update() each cycle */

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

static uint32_t current_loop_pos_us(int64_t now_us)
{
    if (s_loop_duration_us == 0) return 0;
    int64_t elapsed = now_us - s_play_origin_us;
    if (elapsed < 0) elapsed = 0;
    return (uint32_t)(elapsed % s_loop_duration_us);
}

static void start_recording(int track)
{
    if (track < 0 || track >= NUM_TRACKS) track = 0;
    s_recording = true;
    s_record_track = track;
    clear_track_internal(track); /* overwrite policy */

    int64_t now_us = esp_timer_get_time();

    if (s_loop_duration_us == 0) {
        /* First take: clock starts now. Playback isn't running yet. */
        s_record_start_us = now_us;
        s_last_loop_pos_us = 0;
    } else {
        /* Overdub onto existing timeline. Force playback on so the
         * loop position keeps advancing while we capture. If we were
         * paused, resume from the pause position so the user records
         * relative to where they last heard. */
        if (!s_playing) {
            s_play_origin_us = now_us - (int64_t)s_pause_loop_pos_us;
            s_playing = true;
        }
        s_last_loop_pos_us = current_loop_pos_us(now_us);
    }

    ESP_LOGI(TAG, "REC start track=%d loop_len_us=%u",
             s_record_track, (unsigned)s_loop_duration_us);
}

static void stop_recording(void)
{
    if (!s_recording) return;
    s_recording = false;

    int64_t now_us = esp_timer_get_time();

    if (s_loop_duration_us == 0) {
        /* First take: this stop sets the loop length. */
        int64_t dur = now_us - s_record_start_us;
        if (dur <= 1000) {
            /* < 1 ms: treat as an empty take. */
            s_loop_duration_us = 0;
            s_playing = false;
            ESP_LOGI(TAG, "REC stop: empty take");
            return;
        }
        if (dur > MAX_LOOP_US) dur = MAX_LOOP_US;

        s_loop_duration_us = (uint32_t)dur;
        s_play_origin_us = now_us;          /* loop pos 0 = now */
        s_last_loop_pos_us = 0;
        s_pause_loop_pos_us = 0;
        s_playing = true;
        ESP_LOGI(TAG, "REC stop: first loop set (%u us, %.2fs)",
                 (unsigned)s_loop_duration_us, s_loop_duration_us / 1.0e6f);
    } else {
        /* Existing loop stays fixed-length; keep playback running. */
        s_playing = true;
        ESP_LOGI(TAG, "REC stop: overdub track=%d (loop %u us)",
                 s_record_track, (unsigned)s_loop_duration_us);
    }
}

bool loop_recorder_init(void)
{
    memset(s_tracks, 0, sizeof(s_tracks));
    memset(s_event_count, 0, sizeof(s_event_count));

    s_recording = false;
    s_playing = false;
    s_record_track = 0;
    s_record_start_us = 0;
    s_play_origin_us = 0;
    s_pause_loop_pos_us = 0;
    s_loop_duration_us = 0;
    s_last_loop_pos_us = 0;

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
    ESP_LOGI(TAG, "Init: %d tracks x %d events = %u KB PSRAM",
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
    s_loop_duration_us = 0;
    s_last_loop_pos_us = 0;
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

    int64_t now_us = esp_timer_get_time();
    uint32_t t_us;

    if (s_loop_duration_us == 0) {
        /* First take: time is "ms since record start", capped at MAX_LOOP_US. */
        int64_t dt = now_us - s_record_start_us;
        if (dt < 0) dt = 0;
        if (dt >= MAX_LOOP_US) dt = MAX_LOOP_US - 1;
        t_us = (uint32_t)dt;
    } else {
        /* Overdub: record at current loop position. */
        t_us = current_loop_pos_us(now_us);
    }

    s_tracks[s_record_track][count].t_us = t_us;
    s_tracks[s_record_track][count].evt = *evt;
    s_event_count[s_record_track] = count + 1;
}

/* Decide whether a recorded event with timestamp t_us falls inside the
 * just-elapsed playback window (prev_us, cur_us]. Handles wrap-around
 * when the playhead crosses the loop end. */
static inline bool window_contains(uint32_t t_us, uint32_t prev_us,
                                   uint32_t cur_us, bool wrapped)
{
    if (!wrapped) {
        return (t_us > prev_us) && (t_us <= cur_us);
    }
    return (t_us > prev_us) || (t_us <= cur_us);
}

void loop_recorder_update(shared_state_t *snap)
{
    if (snap == NULL) return;

    int64_t now_us = esp_timer_get_time();

    /* CLEAR active track. */
    if (snap->clear_pressed) {
        loop_recorder_clear_track(snap->active_track);
        ESP_LOGI(TAG, "Track %d cleared", snap->active_track);
    }

    /* PLAY / PAUSE — toggle, with pause/resume preserving loop position. */
    if (snap->play_pause_pressed) {
        if (s_loop_duration_us > 0 && loop_recorder_has_data()) {
            if (s_playing) {
                /* Capture position so resume picks up where we left off. */
                s_pause_loop_pos_us = current_loop_pos_us(now_us);
                s_playing = false;
                ESP_LOGI(TAG, "Playback PAUSE @ %u us", (unsigned)s_pause_loop_pos_us);
            } else {
                /* Re-anchor s_play_origin_us so the loop position equals
                 * the saved pause position right now. */
                s_play_origin_us = now_us - (int64_t)s_pause_loop_pos_us;
                s_last_loop_pos_us = s_pause_loop_pos_us;
                s_playing = true;
                ESP_LOGI(TAG, "Playback ON  @ %u us", (unsigned)s_pause_loop_pos_us);
            }
        } else {
            ESP_LOGI(TAG, "Play/pause ignored (no loop)");
        }
    }

    /* RECORD toggle. */
    if (snap->record_pressed) {
        if (!s_recording) {
            start_recording(snap->active_track);
        } else {
            stop_recording();
        }
    }

    /* First-take auto-stop: bound the first loop to MAX_LOOP_US so the
     * tape can't run forever if the user forgets to stop. */
    if (s_recording && s_loop_duration_us == 0) {
        int64_t dt = now_us - s_record_start_us;
        if (dt >= MAX_LOOP_US) {
            ESP_LOGW(TAG, "First take hit %d s — auto-stopping",
                     MAX_LOOP_US / 1000000);
            stop_recording();
        }
    }

    /* Playback emit: any event whose timestamp falls inside the elapsed
     * window since the previous update fires now. */
    if (s_playing && s_loop_duration_us > 0) {
        uint32_t cur_pos_us = current_loop_pos_us(now_us);
        uint32_t prev_pos_us = s_last_loop_pos_us;
        bool wrapped = (cur_pos_us < prev_pos_us);

        for (int t = 0; t < NUM_TRACKS; t++) {
            rec_event_t *tape = s_tracks[t];
            int count = s_event_count[t];
            if (tape == NULL || count <= 0) continue;

            for (int i = 0; i < count; i++) {
                if (window_contains(tape[i].t_us, prev_pos_us, cur_pos_us, wrapped)) {
                    note_event_t e = tape[i].evt;
                    e.source = 20; /* loop playback provenance */
                    note_event_post(e.slot, e.velocity, e.speed, e.loop, e.source);
                }
            }
        }

        s_last_loop_pos_us = cur_pos_us;
    }

    /* Publish transport state (length/position in milliseconds for logs). */
    snap->is_recording = s_recording;
    snap->is_playing = s_playing;
    snap->loop_length = (int)(s_loop_duration_us / 1000);
    snap->playhead = (int)((s_playing ? s_last_loop_pos_us : s_pause_loop_pos_us) / 1000);
}

int loop_recorder_get_length(void)
{
    /* Return milliseconds for callers (was frames previously). */
    return (int)(s_loop_duration_us / 1000);
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
        s_record_start_us = 0;
        s_play_origin_us = 0;
        s_pause_loop_pos_us = 0;
        s_loop_duration_us = 0;
        s_last_loop_pos_us = 0;
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
    s_record_start_us = 0;
    s_play_origin_us = 0;
    s_pause_loop_pos_us = 0;
    s_loop_duration_us = 0;
    s_last_loop_pos_us = 0;
}
