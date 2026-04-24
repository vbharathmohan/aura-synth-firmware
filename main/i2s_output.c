/**
 * i2s_output.c — I2S DMA output driver.
 *
 * Uses the ESP-IDF v6.0 i2s_std API (not the deprecated driver/i2s.h).
 * Allocates the interleaved output buffer in DMA-capable memory.
 */

#include "i2s_output.h"
#include "audio_block.h"

#include "freertos/FreeRTOS.h"
#include "driver/i2s_std.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "i2s_output";

/* ------------------------------------------------------------------ */
/* Static state                                                        */
/* ------------------------------------------------------------------ */

static i2s_chan_handle_t s_tx_chan = NULL;

/* ------------------------------------------------------------------ */
/* Init                                                                */
/* ------------------------------------------------------------------ */

bool i2s_output_init(void)
{
    /* Channel config */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = I2S_DMA_BUF_COUNT;
    chan_cfg.dma_frame_num = BLOCK_SAMPLES;

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_chan, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(err));
        return false;
    }

    /* Standard Philips mode config */
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT,
                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)I2S_PIN_BCLK,
            .ws   = (gpio_num_t)I2S_PIN_WS,
            .dout = (gpio_num_t)I2S_PIN_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(s_tx_chan, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    err = i2s_channel_enable(s_tx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "I2S initialized: %d Hz, 16-bit stereo, "
             "BCLK=%d WS=%d DOUT=%d, DMA bufs=%d × %d frames",
             SAMPLE_RATE, I2S_PIN_BCLK, I2S_PIN_WS, I2S_PIN_DOUT,
             I2S_DMA_BUF_COUNT, BLOCK_SAMPLES);

    return true;
}

/* ------------------------------------------------------------------ */
/* Write                                                               */
/* ------------------------------------------------------------------ */

size_t i2s_output_write(const int16_t *data, size_t bytes)
{
    if (s_tx_chan == NULL) return 0;

    size_t written = 0;
    esp_err_t err = i2s_channel_write(s_tx_chan, data, bytes,
                                       &written, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2s_channel_write error: %s", esp_err_to_name(err));
    }
    return written;
}

/* ------------------------------------------------------------------ */
/* Deinit                                                              */
/* ------------------------------------------------------------------ */

void i2s_output_deinit(void)
{
    if (s_tx_chan != NULL) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
        ESP_LOGI(TAG, "I2S deinitialized");
    }
}
