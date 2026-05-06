/**
 * sensor_task.h — VL53L0X ToF array → note events.
 *
 * Eight VL53L0X sensors are addressed via a 74HC595 shift register on
 * their XSHUT lines so each one can be brought up sequentially and
 * given a unique I2C address.
 *
 * Each sensor maps to a note in the C-major scale (C D E F G A B C).
 * A "downward swipe" — hand approaching the sensor with sufficient
 * velocity — produces a note_event_post() call:
 *     slot     = sampler slot for the currently-active instrument
 *     velocity = 0..255, scaled from approach speed (mm/s)
 *     speed   = pitch ratio for that scale degree
 *
 * The whole task is a producer for the global g_note_queue declared
 * in shared_state.h — it never touches the sampler directly. This is
 * the seam the loop-recorder will plug into later: the recorder will
 * listen to (or replay) the same queue.
 *
 * USAGE:
 *   sensor_task_init();      // power, I2C, shift register, VL53L0Xs
 *   sensor_task_start();     // launch FreeRTOS task on Core 1
 */

#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** One-time hardware bring-up. Returns true if at least one sensor
 *  initialised. Logs which sensors failed. Call from app_main() AFTER
 *  shared_state_init() (because the queue must exist). */
bool sensor_task_init(void);

/** Launch the polling task. Pinned to Core 1 alongside the audio task
 *  (audio is pre-empted by the higher-priority audio task; sensor
 *  polling at 22 ms cadence has no real-time deadline).
 *  Safe to call only once. */
void sensor_task_start(void);

/** Returns the most recent distance in mm for sensor `idx` (0..7).
 *  Returns 0 if the sensor failed to initialise.
 *  Useful for diagnostics / future LED feedback. */
uint16_t sensor_task_last_distance(int idx);

/** Age (ms) of the most recent successful measurement for sensor `idx`.
 *  Large values mean stale data (e.g. no hand present / timeout / sensor
 *  not ready yet). Returns INT32_MAX on invalid index. */
int32_t sensor_task_last_age_ms(int idx);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_TASK_H */
