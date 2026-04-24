/**
 * audio_task.h — Core 1 audio pipeline runner.
 *
 * The audio task is the heart of the instrument. Each cycle:
 *   1. Snapshot shared state from Core 0
 *   2. Run loop recorder (record/playback control frames)
 *   3. Run mixer (synth voices + sampler + effects → mixed block)
 *   4. Convert to I2S format and write to DMA
 *
 * This is the ONLY task on Core 1 — it gets maximum CPU time.
 */

#ifndef AUDIO_TASK_H
#define AUDIO_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * The FreeRTOS task function. Pin to Core 1, highest priority.
 *
 *   xTaskCreatePinnedToCore(audio_task_run, "audio", 8192,
 *                           NULL, configMAX_PRIORITIES-1, NULL, 1);
 */
void audio_task_run(void *param);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_TASK_H */
