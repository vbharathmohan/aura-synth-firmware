/**
 * led_task.h — Aura-Synth front-panel LEDs.
 *
 * Drives a single WS2812 strip (default: 60 LEDs on GPIO 5) with two
 * overlaid layers:
 *
 *   1. Per-sensor "bin" feedback. The strip is divided into 8 bins of
 *      LEDS_PER_BIN pixels each. Bin `b` brightens as the hand
 *      approaches ToF sensor `b`, with a stable per-bin hue so the
 *      array reads as a rainbow scale.
 *
 *   2. Metronome flash on the spare tail of the strip (the last 4
 *      LEDs, which fall outside the 56-LED bin region for a 60-LED
 *      strip). Fast attack, exponential decay, fixed BPM.
 *
 * The task only reads sensor data via sensor_task_last_distance(), so
 * it has zero coupling with the audio path. Safe to start at any time
 * after sensor_task_init() succeeds (or even before — bins simply stay
 * dark until distances arrive).
 *
 * USAGE:
 *   led_task_init();    // RMT setup + clears the strip
 *   led_task_start();   // launches render task + metronome task
 */

#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Compile-time configuration                                          */
/* ------------------------------------------------------------------ */

/* WS2812 data line. Adjust to match your wiring. */
#ifndef LED_STRIP_GPIO
#define LED_STRIP_GPIO          5
#endif

/* Total LEDs on the strip. Bins are LED_STRIP_COUNT / 8 per ToF sensor;
 * any leftover at the end is reserved for the metronome overlay. */
#ifndef LED_STRIP_COUNT
#define LED_STRIP_COUNT         60
#endif

/* Default tempo for the metronome flash. Override at compile time. */
#ifndef METRONOME_BPM
#define METRONOME_BPM           128
#endif

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** One-time hardware bring-up (RMT channel + initial blank). Returns
 *  false if the led_strip driver fails to install. */
bool led_task_init(void);

/** Launch the render + metronome tasks (Core 1, low priority). Safe to
 *  call only once. */
void led_task_start(void);

/** Update tempo at runtime (BPM). 30..240 are reasonable. 0 disables
 *  the metronome (LEDs in the spare row stay dark). */
void led_task_set_bpm(int bpm);

/* ------------------------------------------------------------------ */
/* Boot-time status rendering (call before led_task_start)             */
/* ------------------------------------------------------------------ */

/** Clear strip immediately (all LEDs off). */
void led_task_boot_clear(void);

/** Light one ToF bin as ready/not-ready during startup.
 *  ready=true  -> green chunk
 *  ready=false -> red chunk */
void led_task_boot_set_sensor_ready(int sensor_idx, bool ready);

/** Flash entire strip white, then return to previous state. */
void led_task_boot_flash_white(uint8_t level, int duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* LED_TASK_H */
