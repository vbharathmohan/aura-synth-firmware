/**
 * main.c — Aura-Synth boot + mode controller.
 *
 * The instrument has two top-level modes, selected at compile time:
 *
 *   DEMO_MODE
 *       No physical buttons are wired up. A small task on Core 0
 *       cycles the active instrument (piano → steel drum → trumpet →
 *       808 bass) every 10 seconds. The 8 ToF sensors play whatever
 *       instrument is currently selected, so a swipe across the array
 *       walks through one octave of C-major in the active instrument.
 *
 *   INTEGRATION_MODE
 *       Four physical buttons select instruments OR drum pads (a
 *       fifth button toggles between the two functions). Three
 *       master-bus controls map to volume / biquad filter cutoff /
 *       delay mix. ToFs always play the currently selected instrument.
 *
 * Only DEMO_MODE is fully implemented today — INTEGRATION_MODE has
 * scaffolding (button task, pin map, dial reads) so it compiles and
 * is easy to flesh out next. The seam in both modes is the same:
 * everything that produces sound goes through the note-event queue,
 * which keeps the loop-recorder feature (planned next) trivial to
 * slot in later.
 *
 * To switch modes: comment/uncomment the #define below.
 */

/* ====================== MODE SELECT ============================== */
#define DEMO_MODE
/* #define INTEGRATION_MODE */
/* ================================================================== */

#if defined(DEMO_MODE) && defined(INTEGRATION_MODE)
#error "Pick exactly one of DEMO_MODE / INTEGRATION_MODE"
#endif
#if !defined(DEMO_MODE) && !defined(INTEGRATION_MODE)
#error "Pick exactly one of DEMO_MODE / INTEGRATION_MODE"
#endif

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"

#include "shared_state.h"
#include "audio_block.h"
#include "i2s_output.h"
#include "sampler.h"
#include "mixer.h"
#include "loop_recorder.h"
#include "audio_task.h"
#include "effects.h"
#include "sensor_task.h"
#include "led_task.h"

static const char *TAG = "aura_synth";

/* ------------------------------------------------------------------ */
/* Embedded WAV samples                                                */
/* ------------------------------------------------------------------ */
/* Each entry below corresponds to a CMakeLists `target_add_binary_data`
 * line. The asm() label is fixed by the linker (`_binary_<file>_start`);
 * the C identifier on the left can be anything legal — we use a friendly
 * name so files starting with a digit (808_c1.wav) compile cleanly. */

/* Drum kit */
extern const uint8_t kick_wav_start[]            asm("_binary_kick_wav_start");
extern const uint8_t kick_wav_end[]              asm("_binary_kick_wav_end");
extern const uint8_t snare_wav_start[]           asm("_binary_snare_wav_start");
extern const uint8_t snare_wav_end[]             asm("_binary_snare_wav_end");
extern const uint8_t hihat_wav_start[]           asm("_binary_hihat_wav_start");
extern const uint8_t hihat_wav_end[]             asm("_binary_hihat_wav_end");
extern const uint8_t clap_wav_start[]           asm("_binary_clap_wav_start");
extern const uint8_t clap_wav_end[]             asm("_binary_clap_wav_end");

/* Melodic instruments (each is one octave of C natural) */
extern const uint8_t piano_c4_wav_start[]        asm("_binary_piano_c4_wav_start");
extern const uint8_t piano_c4_wav_end[]          asm("_binary_piano_c4_wav_end");
extern const uint8_t steel_drum_c4_wav_start[]   asm("_binary_steel_drum_c4_wav_start");
extern const uint8_t steel_drum_c4_wav_end[]     asm("_binary_steel_drum_c4_wav_end");
extern const uint8_t trumpet_c6_wav_start[]      asm("_binary_trumpet_c6_wav_start");
extern const uint8_t trumpet_c6_wav_end[]        asm("_binary_trumpet_c6_wav_end");
extern const uint8_t bass_808_wav_start[]        asm("_binary_808_c1_wav_start");
extern const uint8_t bass_808_wav_end[]          asm("_binary_808_c1_wav_end");

/* ------------------------------------------------------------------ */
/* Master delay effect (allocated once; toggled via mix knob)          */
/* ------------------------------------------------------------------ */

static delay_t s_master_delay;
static bool    s_master_delay_ready = false;

/* ------------------------------------------------------------------ */
/* Sampler registration                                                */
/* ------------------------------------------------------------------ */

static void register_all_samples(void)
{
    /* Slot indices live in shared_state.h so sensor_task and mixer
     * agree on the mapping. */
    sampler_register(SAMPLE_SLOT_PIANO,
                     "piano",     piano_c4_wav_start,      piano_c4_wav_end);
    sampler_register(SAMPLE_SLOT_STEEL_DRUM,
                     "steeldrm",  steel_drum_c4_wav_start, steel_drum_c4_wav_end);
    sampler_register(SAMPLE_SLOT_TRUMPET,
                     "trumpet",   trumpet_c6_wav_start,    trumpet_c6_wav_end);
    sampler_register(SAMPLE_SLOT_808_BASS,
                     "808bass",   bass_808_wav_start,      bass_808_wav_end);
    sampler_register(SAMPLE_SLOT_KICK,
                     "kick",      kick_wav_start,          kick_wav_end);
    sampler_register(SAMPLE_SLOT_SNARE,
                     "snare",     snare_wav_start,         snare_wav_end);
    sampler_register(SAMPLE_SLOT_HIHAT,
                     "hihat",     hihat_wav_start,         hihat_wav_end);
    sampler_register(SAMPLE_SLOT_CLAP,
                     "clap",      clap_wav_start,          clap_wav_end);
}

/* ================================================================== */
/* DEMO MODE                                                           */
/* ================================================================== */
#ifdef DEMO_MODE

#define DEMO_INSTRUMENT_PERIOD_MS   10000   /* 10 s per instrument */

static void demo_mode_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "demo_mode_task on core %d — cycling instruments every %d ms",
             xPortGetCoreID(), DEMO_INSTRUMENT_PERIOD_MS);

    const instrument_t cycle[] = {
        INST_PIANO, INST_STEEL_DRUM, INST_TRUMPET, INST_808_BASS,
    };
    const char *names[] = {"piano", "steel drum", "trumpet", "808 bass"};
    const int N = sizeof(cycle) / sizeof(cycle[0]);

    int idx = 0;
    while (1) {
        if (shared_state_lock()) {
            g_state.active_instrument = cycle[idx];
            shared_state_unlock();
        }
        ESP_LOGI(TAG, "==> Instrument: %s", names[idx]);

        vTaskDelay(pdMS_TO_TICKS(DEMO_INSTRUMENT_PERIOD_MS));
        idx = (idx + 1) % N;
    }
}

#endif /* DEMO_MODE */

/* ================================================================== */
/* INTEGRATION MODE                                                    */
/* ================================================================== */
#ifdef INTEGRATION_MODE

/* These pin assignments are placeholders — fill in once the front
 * panel is wired. The user reserved pins 12, 13, 14 for the shift
 * register and the I2C / I2S pins are fixed; everything else on
 * the ESP32 GPIO map is fair game. */
#define BTN_PAD_0_PIN           GPIO_NUM_15  /* piano  / kick  */
#define BTN_PAD_1_PIN           GPIO_NUM_16  /* steel  / hihat */
#define BTN_PAD_2_PIN           GPIO_NUM_17  /* trump  / clap  */
#define BTN_PAD_3_PIN           GPIO_NUM_18  /* 808    / snare */
#define BTN_PAD_TOGGLE_PIN      GPIO_NUM_19  /* instr <-> drums */
/* Prompt 2 transport buttons (active-low, pull-up enabled). */
#define BTN_REC_PIN             GPIO_NUM_21  /* record / stop recording */
#define BTN_CLEAR_PIN           GPIO_NUM_23  /* clear active track */
#define BTN_PLAY_PAUSE_PIN      GPIO_NUM_27  /* play / pause */
/* Prompt 3: single button cycles active track 0->1->2->3->0 */
#define BTN_TRACK_CYCLE_PIN     GPIO_NUM_32

/* Master FX controls (analog reads / encoder) */
#define DIAL_VOLUME_ADC         ADC1_CHANNEL_0  /* placeholder */
#define DIAL_FILTER_ADC         ADC1_CHANNEL_3  /* placeholder */
#define DIAL_DELAY_ADC          ADC1_CHANNEL_6  /* placeholder */

#define BTN_DEBOUNCE_US         50000

static const int s_pad_pins[NUM_DRUM_PADS] = {
    BTN_PAD_0_PIN, BTN_PAD_1_PIN, BTN_PAD_2_PIN, BTN_PAD_3_PIN,
};

/* Drum slots in the same physical order as the buttons.
 * (User-specified mapping: kick / hihat / clap / snare.) */
static const uint8_t s_pad_drum_slot[NUM_DRUM_PADS] = {
    SAMPLE_SLOT_KICK,  SAMPLE_SLOT_HIHAT,
    SAMPLE_SLOT_CLAP,  SAMPLE_SLOT_SNARE,
};

/* Instruments in the same physical order as the buttons. */
static const instrument_t s_pad_instrument[NUM_DRUM_PADS] = {
    INST_PIANO, INST_STEEL_DRUM, INST_TRUMPET, INST_808_BASS,
};

static void integration_buttons_init(void)
{
    gpio_config_t io = {0};
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    io.pin_bit_mask =
        (1ULL << BTN_PAD_0_PIN) | (1ULL << BTN_PAD_1_PIN) |
        (1ULL << BTN_PAD_2_PIN) | (1ULL << BTN_PAD_3_PIN) |
        (1ULL << BTN_PAD_TOGGLE_PIN) |
        (1ULL << BTN_REC_PIN) | (1ULL << BTN_CLEAR_PIN) |
        (1ULL << BTN_PLAY_PAUSE_PIN) |
        (1ULL << BTN_TRACK_CYCLE_PIN);
    gpio_config(&io);
}

static void integration_mode_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "integration_mode_task on core %d", xPortGetCoreID());
    ESP_LOGW(TAG, "INTEGRATION_MODE button & dial handling is a stub.");

    integration_buttons_init();

    bool prev_pad[NUM_DRUM_PADS] = {true, true, true, true};
    bool prev_toggle = true;
    bool prev_rec = true;
    bool prev_clear = true;
    bool prev_play = true;
    bool prev_track = true;
    int64_t last_edge_us[NUM_DRUM_PADS + 5] = {0}; /* pads + toggle + rec/clear/play/track */

    while (1) {
        int64_t now = (int64_t)xTaskGetTickCount() * portTICK_PERIOD_MS * 1000;

        /* Read the toggle button (pad mode switch) */
        bool toggle_now = gpio_get_level(BTN_PAD_TOGGLE_PIN) != 0;
        if (!toggle_now && prev_toggle &&
            (now - last_edge_us[NUM_DRUM_PADS]) > BTN_DEBOUNCE_US) {
            if (shared_state_lock()) {
                g_state.pad_mode = (g_state.pad_mode == PAD_INSTRUMENTS)
                                    ? PAD_DRUMS : PAD_INSTRUMENTS;
                shared_state_unlock();
            }
            last_edge_us[NUM_DRUM_PADS] = now;
            ESP_LOGI(TAG, "Pad mode -> %s",
                     g_state.pad_mode == PAD_DRUMS ? "DRUMS" : "INSTRUMENTS");
        }
        prev_toggle = toggle_now;

        /* Transport buttons (Prompt 2) */
        bool rec_now = gpio_get_level(BTN_REC_PIN) != 0;
        if (!rec_now && prev_rec &&
            (now - last_edge_us[NUM_DRUM_PADS + 1]) > BTN_DEBOUNCE_US) {
            if (shared_state_lock()) {
                g_state.record_pressed = true;
                shared_state_unlock();
            }
            last_edge_us[NUM_DRUM_PADS + 1] = now;
            ESP_LOGI(TAG, "Transport: REC toggle");
        }
        prev_rec = rec_now;

        bool clr_now = gpio_get_level(BTN_CLEAR_PIN) != 0;
        if (!clr_now && prev_clear &&
            (now - last_edge_us[NUM_DRUM_PADS + 2]) > BTN_DEBOUNCE_US) {
            if (shared_state_lock()) {
                g_state.clear_pressed = true;
                shared_state_unlock();
            }
            last_edge_us[NUM_DRUM_PADS + 2] = now;
            ESP_LOGI(TAG, "Transport: CLEAR");
        }
        prev_clear = clr_now;

        bool play_now = gpio_get_level(BTN_PLAY_PAUSE_PIN) != 0;
        if (!play_now && prev_play &&
            (now - last_edge_us[NUM_DRUM_PADS + 3]) > BTN_DEBOUNCE_US) {
            if (shared_state_lock()) {
                g_state.play_pause_pressed = true;
                shared_state_unlock();
            }
            last_edge_us[NUM_DRUM_PADS + 3] = now;
            ESP_LOGI(TAG, "Transport: PLAY/PAUSE");
        }
        prev_play = play_now;

        /* Prompt 3: cycle active track with one button */
        bool track_now = gpio_get_level(BTN_TRACK_CYCLE_PIN) != 0;
        if (!track_now && prev_track &&
            (now - last_edge_us[NUM_DRUM_PADS + 4]) > BTN_DEBOUNCE_US) {
            int next_track = 0;
            if (shared_state_lock()) {
                next_track = (g_state.active_track + 1) % NUM_TRACKS;
                g_state.active_track = (uint8_t)next_track;
                shared_state_unlock();
            }
            last_edge_us[NUM_DRUM_PADS + 4] = now;
            ESP_LOGI(TAG, "Transport: TRACK -> %d", next_track);
        }
        prev_track = track_now;

        /* Read the four pad buttons */
        for (int i = 0; i < NUM_DRUM_PADS; i++) {
            bool now_pressed = gpio_get_level(s_pad_pins[i]) == 0; /* active-low */
            bool was_pressed = !prev_pad[i];

            if (now_pressed && !was_pressed &&
                (now - last_edge_us[i]) > BTN_DEBOUNCE_US) {
                if (g_state.pad_mode == PAD_DRUMS) {
                    /* Trigger drum directly */
                    note_event_post(s_pad_drum_slot[i],
                                    /*velocity=*/200,
                                    /*speed=*/1.0f,
                                    /*loop=*/false,
                                    /*source=*/(int8_t)(10 + i));
                } else {
                    /* Switch active instrument */
                    if (shared_state_lock()) {
                        g_state.active_instrument = s_pad_instrument[i];
                        shared_state_unlock();
                    }
                    ESP_LOGI(TAG, "Active instrument -> %d", (int)s_pad_instrument[i]);
                }
                last_edge_us[i] = now;
            }
            prev_pad[i] = !now_pressed;
        }

        /* TODO: read ADC dials and update master_volume / master_filter /
         * master_delay_mix in g_state. Until that lands the master
         * effects use whatever shared_state_init() set. */

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#endif /* INTEGRATION_MODE */

/* ================================================================== */
/* Master FX wiring                                                    */
/* ================================================================== */
/* The mixer reads master_volume directly each cycle. The biquad
 * filter is wired as a master FX in slot 0; we attach the global
 * `master_filter` cutoff here (the value can be changed later from
 * either mode without re-wiring). */

static biquad_t s_master_biquad;

static void master_fx_init(void)
{
    biquad_init_lpf(&s_master_biquad,
                    /*cutoff=*/2000.0f,
                    /*q=*/0.707f,
                    /*sample_rate=*/(float)SAMPLE_RATE);
    mixer_set_master_fx(0, fx_biquad, &s_master_biquad);

    size_t psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    if (psram > 0) {
        if (delay_init(&s_master_delay,
                       /*delay_ms=*/280.0f,
                       /*feedback=*/0.35f,
                       /*mix=*/0.0f,           /* start dry */
                       /*sr=*/(float)SAMPLE_RATE)) {
            s_master_delay_ready = true;
            mixer_set_master_fx(1, fx_delay, &s_master_delay);
        }
    } else {
        ESP_LOGW(TAG, "No PSRAM detected — delay effect disabled");
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "================================================");
#ifdef DEMO_MODE
    ESP_LOGI(TAG, "  AURA-SYNTH — DEMO_MODE (ToF + cycling instr.)");
#else
    ESP_LOGI(TAG, "  AURA-SYNTH — INTEGRATION_MODE");
#endif
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "PSRAM: %lu KB",
             (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    /* 1. Shared state (creates the mutex AND the note-event queue) */
    shared_state_init();
    ESP_LOGI(TAG, "[1/8] Shared state OK");

    /* 2. Block pool */
    if (!audio_pool_init()) { ESP_LOGE(TAG, "Block pool failed"); return; }
    ESP_LOGI(TAG, "[2/8] Block pool OK");

    /* 3. I2S */
    if (!i2s_output_init()) { ESP_LOGE(TAG, "I2S failed"); return; }
    ESP_LOGI(TAG, "[3/8] I2S OK");

    /* 4. Sampler + WAVs */
    sampler_init();
    register_all_samples();
    ESP_LOGI(TAG, "[4/8] Sampler OK");

    /* 5. Mixer + master FX */
    mixer_init((float)SAMPLE_RATE);
    master_fx_init();
    ESP_LOGI(TAG, "[5/8] Mixer OK (master FX wired%s)",
             s_master_delay_ready ? ", delay armed" : "");

    /* 6. Loop recorder (PSRAM-only). Future feature; safe to init now. */
    if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) > 0) {
        loop_recorder_init();
    }
    ESP_LOGI(TAG, "[6/8] Loop recorder OK");

    /* 7. LED hardware (needed before sensor init so ready status can
     * illuminate per-sensor chunks during startup). */
    if (!led_task_init()) {
        ESP_LOGW(TAG, "[7/8] LEDs NOT ready");
    } else {
        led_task_boot_clear();
        ESP_LOGI(TAG, "[7/8] LEDs OK");
    }

    /* 8. Sensors. During init each "ToF ready" log lights one LED chunk.
     * After all sensors are processed: full-strip white flash then clear. */
    if (!sensor_task_init()) {
        ESP_LOGW(TAG, "[8/8] Sensors NOT ready — silent gesture input");
    } else {
        led_task_boot_flash_white(120, 250);
        led_task_boot_clear();
        sensor_task_start();
        ESP_LOGI(TAG, "[8/8] Sensors OK");
    }

    /* Start normal LED rendering once boot animation is complete. */
    led_task_start();

    /* Audio pipeline on Core 1 — highest priority */
    xTaskCreatePinnedToCore(audio_task_run, "audio", 8192, NULL,
                            configMAX_PRIORITIES - 1, NULL, 1);

#ifdef DEMO_MODE
    xTaskCreatePinnedToCore(demo_mode_task, "demo_cycle", 3072, NULL,
                            4, NULL, 0);
#else
    xTaskCreatePinnedToCore(integration_mode_task, "integ", 4096, NULL,
                            4, NULL, 0);
#endif

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Boot complete — swipe across the ToF array to play.");
}
