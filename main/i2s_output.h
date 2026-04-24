/**
 * i2s_output.h — I2S DMA output driver for Aura-Synth.
 *
 * Wraps the ESP-IDF v6.0 I2S standard mode API.
 * Configured for 16-bit stereo output at the project sample rate.
 *
 * USAGE:
 *   i2s_output_init();                     // call once at boot
 *   i2s_output_write(i2s_buf, num_bytes);  // from audio task
 */

#ifndef I2S_OUTPUT_H
#define I2S_OUTPUT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

/**
 * Sample rate in Hz.
 * 44100 = CD quality. Must match the rate your WAV samples were
 * converted to, and the rate your synth voices generate at.
 */
#define SAMPLE_RATE         44100

/**
 * GPIO pin assignments for I2S.
 * Change these to match your wiring to the DAC board.
 */
#define I2S_PIN_BCLK        26    /* bit clock */
#define I2S_PIN_WS          25    /* word select (L/R clock) */
#define I2S_PIN_DOUT        22    /* data out to DAC */

/**
 * Number of DMA buffers. More buffers = more latency tolerance
 * but uses more memory. 4 is a safe default.
 */
#define I2S_DMA_BUF_COUNT   4

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * Initialize the I2S driver in standard Philips mode.
 * Configures: 44100 Hz, 16-bit, stereo, DMA.
 * Returns true on success.
 */
bool i2s_output_init(void);

/**
 * Write interleaved int16 stereo samples to I2S.
 * Blocks until the DMA buffer accepts the data.
 *
 * @param data      Interleaved L/R int16 samples
 * @param bytes     Total bytes to write (BLOCK_SAMPLES * 2 * sizeof(int16_t))
 * @return          Number of bytes actually written
 */
size_t i2s_output_write(const int16_t *data, size_t bytes);

/**
 * Disable and free the I2S channel. Call on shutdown (optional).
 */
void i2s_output_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* I2S_OUTPUT_H */
