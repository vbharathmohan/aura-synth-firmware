#include "telemetry_uart.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "shared_state.h"
#include "loop_recorder.h"
#include "audio_scope.h"

static const char *TAG = "telemetry_uart";

#define TEL_UART_PORT         UART_NUM_1
#define TEL_UART_TX_PIN       GPIO_NUM_8   /* user-approved dedicated TX pin */
#define TEL_BAUD              115200
#define TEL_PERIOD_MS         50           /* 20 Hz */
#define TEL_DETAIL_DIVIDER    2            /* heavy chunks at 10 Hz */

#define TEL_BUCKETS           64
#define TEL_SCOPE_POINTS      64

static TaskHandle_t s_task;
static bool s_inited;

static inline char hex_nibble(uint8_t v)
{
    v &= 0x0F;
    return (v < 10) ? (char)('0' + v) : (char)('A' + (v - 10));
}

static void encode_track_buckets_hex(char *dst, const uint8_t *buckets, int n)
{
    for (int i = 0; i < n; i++) {
        dst[i] = hex_nibble(buckets[i]);
    }
    dst[n] = '\0';
}

/* Encode signed waveform to 0..255 hex bytes (2 chars per point). */
static void encode_scope_hex(char *dst, const int16_t *scope, int n)
{
    for (int i = 0; i < n; i++) {
        int v = ((int)scope[i] / 256) + 128;
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        uint8_t b = (uint8_t)v;
        dst[i * 2 + 0] = hex_nibble((uint8_t)(b >> 4));
        dst[i * 2 + 1] = hex_nibble((uint8_t)(b & 0x0F));
    }
    dst[n * 2] = '\0';
}

static void telemetry_task(void *param)
{
    (void)param;

    uint8_t trk[NUM_TRACKS][TEL_BUCKETS] = {};
    int16_t scope_raw[256] = {};
    int16_t scope_ds[TEL_SCOPE_POINTS] = {};

    char t0[TEL_BUCKETS + 1];
    char t1[TEL_BUCKETS + 1];
    char t2[TEL_BUCKETS + 1];
    char t3[TEL_BUCKETS + 1];
    char sh[TEL_SCOPE_POINTS * 2 + 1];
    char line[1024];
    uint32_t tick = 0;

    while (1) {
        shared_state_t snap = {0};
        if (shared_state_lock()) {
            snap = g_state; /* do not consume one-shot flags */
            shared_state_unlock();
        }

        for (int t = 0; t < NUM_TRACKS; t++) {
            loop_recorder_render_track_buckets(t, trk[t], TEL_BUCKETS);
        }

        int n = audio_scope_read_i16(scope_raw, 256);
        if (n <= 0) {
            memset(scope_ds, 0, sizeof(scope_ds));
        } else {
            /* Downsample to 64 points by averaging bins. */
            for (int i = 0; i < TEL_SCOPE_POINTS; i++) {
                int a = (i * n) / TEL_SCOPE_POINTS;
                int b = ((i + 1) * n) / TEL_SCOPE_POINTS;
                if (b <= a) b = a + 1;
                if (b > n) b = n;
                int acc = 0;
                int c = 0;
                for (int k = a; k < b; k++) {
                    acc += scope_raw[k];
                    c++;
                }
                scope_ds[i] = (c > 0) ? (int16_t)(acc / c) : 0;
            }
        }

        encode_track_buckets_hex(t0, trk[0], TEL_BUCKETS);
        encode_track_buckets_hex(t1, trk[1], TEL_BUCKETS);
        encode_track_buckets_hex(t2, trk[2], TEL_BUCKETS);
        encode_track_buckets_hex(t3, trk[3], TEL_BUCKETS);
        encode_scope_hex(sh, scope_ds, TEL_SCOPE_POINTS);

        /* Lightweight header (always): AURAH,1,mode,track,instr,wavePct,rec,play,loop_ms,head_ms */
        int len = snprintf(
            line, sizeof(line),
            "AURAH,1,%d,%d,%d,%d,%d,%d,%d,%d\n",
            (int)snap.mode,
            (int)snap.active_track,
            (int)snap.active_instrument,
            (int)(snap.master_waveform_mix * 100.0f),
            snap.is_recording ? 1 : 0,
            snap.is_playing ? 1 : 0,
            snap.loop_length,
            snap.playhead);

        if (len > 0) {
            uart_write_bytes(TEL_UART_PORT, line, len);
        }

        /* Heavy detail chunks less often:
         * AURAT,1,t0,t1,t2,t3
         * AURAS,1,scopeHex
         */
        if ((tick % TEL_DETAIL_DIVIDER) == 0U) {
            int tlen = snprintf(
                line, sizeof(line),
                "AURAT,1,%s,%s,%s,%s\n",
                t0, t1, t2, t3);
            if (tlen > 0) {
                uart_write_bytes(TEL_UART_PORT, line, tlen);
            }

            int slen = snprintf(line, sizeof(line), "AURAS,1,%s\n", sh);
            if (slen > 0) {
                uart_write_bytes(TEL_UART_PORT, line, slen);
            }
        }
        tick++;

        vTaskDelay(pdMS_TO_TICKS(TEL_PERIOD_MS));
    }
}

bool telemetry_uart_init(void)
{
    if (s_inited) {
        return true;
    }

    const uart_config_t cfg = {
        .baud_rate = TEL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(TEL_UART_PORT, &cfg));
    /* TX-only: RX pin is left unchanged/unwired. */
    ESP_ERROR_CHECK(uart_set_pin(TEL_UART_PORT, TEL_UART_TX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(TEL_UART_PORT, 1024, 0, 0, NULL, 0));

    s_inited = true;
    ESP_LOGI(TAG, "UART telemetry ready: port=%d tx=%d baud=%d",
             (int)TEL_UART_PORT, (int)TEL_UART_TX_PIN, TEL_BAUD);
    return true;
}

void telemetry_uart_start(void)
{
    if (!s_inited || s_task != NULL) {
        return;
    }
    xTaskCreatePinnedToCore(telemetry_task, "telemetry_uart", 4096, NULL, 2, &s_task, 0);
}

