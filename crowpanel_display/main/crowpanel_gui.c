#include "crowpanel_gui.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"

static const char *TAG = "crowpanel_gui";

/* ------------------------------------------------------- */
/* Display (kept from known working test firmware)         */
/* ------------------------------------------------------- */
#define LCD_H_RES     800
#define LCD_V_RES     480
#define LCD_PIN_BL    2
#define LCD_PIN_PCLK  0
#define LCD_PIN_VSYNC 41
#define LCD_PIN_HSYNC 39
#define LCD_PIN_DE    40
#define LCD_DATA_PINS { \
    8, 3, 46, 9, 1,     \
    5, 6, 7, 15, 16, 4, \
    45, 48, 47, 21, 14  \
}

/* ------------------------------------------------------- */
/* UART (kept from known working test firmware)            */
/* ------------------------------------------------------- */
#define GUI_UART_PORT    UART_NUM_1
#define GUI_UART_RX_PIN  44
#define GUI_UART_TX_PIN  43
#define GUI_UART_BAUD    921600
#define GUI_UART_BUF     2048

#define TL_BUCKETS    64
#define TL_TRACKS     4
#define SCOPE_POINTS  64

#define BG_COLOR      0x0843 /* dark navy */
#define FG_COLOR      0xE73C /* light text */
#define FRAME_COLOR   0x2108
#define PLAYHEAD_CLR  0xF810 /* pink */
#define SCOPE_COLOR   0x07FF /* cyan */

/* Diagnostic mode: lock all motion to isolate panel scan/timing drift.
 * 1 = fixed-buffer direct draw + frozen playhead/wave
 * 0 = normal telemetry-driven behavior */
#define DISPLAY_DIAGNOSTIC_LOCK 0

typedef struct {
    int mode, track, instr, wave_pct, rec, play, loop_ms, head_ms;
    uint8_t tl[TL_TRACKS][TL_BUCKETS];
    uint8_t scope[SCOPE_POINTS];
    bool valid;
} gui_state_t;

static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_fb;
static uint16_t *s_fb_a;
static uint16_t *s_draw_buf;
#if !DISPLAY_DIAGNOSTIC_LOCK
static uint16_t *s_fb_b;
static bool s_use_fb_a;
#endif
static bool s_ready;
static TaskHandle_t s_uart_task;
static TaskHandle_t s_render_task;
static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static gui_state_t s_state;
static uint32_t s_state_seq;

static uint8_t hex_val(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0;
}

static uint16_t color_for_code(uint8_t code)
{
    switch (code) {
    case 1: return 0x67FA; /* synth */
    case 2: return 0xA27D; /* piano */
    case 3: return 0x64DF; /* steel */
    case 4: return 0xFD20; /* trumpet */
    case 5: return 0xFAD3; /* 808 */
    case 6: return 0xF810; /* kick */
    case 7: return 0xFB40; /* snare */
    case 8: return 0xFF80; /* hihat */
    case 9: return 0x37F3; /* clap */
    default: return 0x2129;
    }
}

static inline void put_pixel(int x, int y, uint16_t c)
{
    if ((unsigned)x < LCD_H_RES && (unsigned)y < LCD_V_RES) {
        s_draw_buf[y * LCD_H_RES + x] = c;
    }
}

static void fill_rect(int x, int y, int w, int h, uint16_t c)
{
    if (w <= 0 || h <= 0) return;
    for (int yy = y; yy < y + h; yy++) {
        for (int xx = x; xx < x + w; xx++) {
            put_pixel(xx, yy, c);
        }
    }
}

static void draw_rect(int x, int y, int w, int h, uint16_t c)
{
    for (int xx = x; xx < x + w; xx++) {
        put_pixel(xx, y, c);
        put_pixel(xx, y + h - 1, c);
    }
    for (int yy = y; yy < y + h; yy++) {
        put_pixel(x, yy, c);
        put_pixel(x + w - 1, yy, c);
    }
}

/* Minimal 5x7 glyphs: enough for M/T/I/W/R/P/L/H/A/U/O/D/E/C/F/S/N/Y/X and digits. */
static const uint8_t GLYPH_NUM[10][7] = {
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, /* 0 */
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, /* 1 */
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}, /* 2 */
    {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}, /* 3 */
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, /* 4 */
    {0x1F,0x10,0x10,0x1E,0x01,0x01,0x1E}, /* 5 */
    {0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E}, /* 6 */
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, /* 7 */
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, /* 8 */
    {0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E}, /* 9 */
};

static uint8_t glyph_row(char ch, int row)
{
    if (row < 0 || row > 6) return 0;
    if (ch >= '0' && ch <= '9') {
        return GLYPH_NUM[ch - '0'][row];
    }
    switch (ch) {
    case 'A': { static const uint8_t g[7]={0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'C': { static const uint8_t g[7]={0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; return g[row]; }
    case 'D': { static const uint8_t g[7]={0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}; return g[row]; }
    case 'E': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return g[row]; }
    case 'F': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'H': { static const uint8_t g[7]={0x11,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[row]; }
    case 'I': { static const uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x1F}; return g[row]; }
    case 'L': { static const uint8_t g[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[row]; }
    case 'M': { static const uint8_t g[7]={0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return g[row]; }
    case 'N': { static const uint8_t g[7]={0x11,0x19,0x19,0x15,0x13,0x13,0x11}; return g[row]; }
    case 'O': { static const uint8_t g[7]={0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'P': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[row]; }
    case 'R': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[row]; }
    case 'S': { static const uint8_t g[7]={0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[row]; }
    case 'T': { static const uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[row]; }
    case 'U': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[row]; }
    case 'W': { static const uint8_t g[7]={0x11,0x11,0x11,0x15,0x15,0x15,0x0A}; return g[row]; }
    case 'X': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}; return g[row]; }
    case 'Y': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return g[row]; }
    case ':': { static const uint8_t g[7]={0x00,0x04,0x04,0x00,0x04,0x04,0x00}; return g[row]; }
    case '%': { static const uint8_t g[7]={0x18,0x19,0x02,0x04,0x08,0x13,0x03}; return g[row]; }
    case '-': { static const uint8_t g[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; return g[row]; }
    case ' ': return 0;
    default: return 0;
    }
}

static void draw_char5x7(int x, int y, char ch, uint16_t c)
{
    for (int r = 0; r < 7; r++) {
        uint8_t bits = glyph_row(ch, r);
        for (int col = 0; col < 5; col++) {
            if (bits & (1u << (4 - col))) {
                put_pixel(x + col, y + r, c);
            }
        }
    }
}

static void draw_char5x7_scaled(int x, int y, char ch, uint16_t c, int scale)
{
    if (scale <= 1) {
        draw_char5x7(x, y, ch, c);
        return;
    }
    for (int r = 0; r < 7; r++) {
        uint8_t bits = glyph_row(ch, r);
        for (int col = 0; col < 5; col++) {
            if (bits & (1u << (4 - col))) {
                fill_rect(x + col * scale, y + r * scale, scale, scale, c);
            }
        }
    }
}

static void draw_text5x7(int x, int y, const char *s, uint16_t c, int scale)
{
    for (int i = 0; s[i] != '\0'; i++) {
        draw_char5x7_scaled(x + i * (5 * scale + scale), y, s[i], c, scale);
    }
}

static const char *mode_name_short(int m)
{
    switch (m) {
    case 0: return "SYNTH";
    case 1: return "DRUMS";
    case 2: return "PIANO";
    case 3: return "FX";
    case 4: return "SAMPLE";
    default: return "UNK";
    }
}

static bool synth_activity_from_tape(const gui_state_t *st, int x_idx, int *strength_out)
{
    int hit = 0;
    for (int tr = 0; tr < TL_TRACKS; tr++) {
        if (st->tl[tr][x_idx] == 1) {
            hit++;
        }
    }
    if (strength_out != NULL) {
        *strength_out = hit;
    }
    return hit > 0;
}

/* Simpler synth waveform math (tape-driven activity + wave mix). */
static void draw_synth_scope(const gui_state_t *st, int sx, int sy, int sw, int sh)
{
    int center = sy + sh / 2;
    float wave_mix = (float)st->wave_pct / 100.0f; /* 0 sine .. 1 saw */
    if (wave_mix < 0.0f) wave_mix = 0.0f;
    if (wave_mix > 1.0f) wave_mix = 1.0f;
    /* Keep waveform shape anchored to X so the UI doesn't appear to scroll.
     * We animate amplitude (not phase) with playhead to preserve motion feel. */
    float amp_pulse = 1.0f;
#if !DISPLAY_DIAGNOSTIC_LOCK
    if (st->play) {
        amp_pulse = 0.90f + 0.10f * sinf((float)st->head_ms * 0.01f);
    }
#endif
    int prev_y = center;
    for (int i = 0; i < SCOPE_POINTS; i++) {
        int x = sx + 2 + (i * (sw - 4)) / (SCOPE_POINTS - 1);
        int idx = (i * TL_BUCKETS) / SCOPE_POINTS;
        if (idx < 0) idx = 0;
        if (idx >= TL_BUCKETS) idx = TL_BUCKETS - 1;

        int strength = 0;
        bool active = synth_activity_from_tape(st, idx, &strength);

        float amp = active ? (34.0f + 12.0f * (float)strength) : 8.0f;
        amp *= amp_pulse;
        float t = ((float)i / (float)(SCOPE_POINTS - 1)) * 6.2831853f;
        float wt = (2.0f + 0.4f * (float)strength) * t;
        float s = sinf(wt);
        float saw = ((float)fmodf(wt, 6.2831853f) /
                     3.14159265f) - 1.0f;
        float v = (1.0f - wave_mix) * s + wave_mix * saw;
        int y = center - (int)(v * amp);
        if (y < sy + 2) y = sy + 2;
        if (y > sy + sh - 3) y = sy + sh - 3;

        if (i > 0) {
            int y0 = (prev_y < y) ? prev_y : y;
            int y1 = (prev_y > y) ? prev_y : y;
            for (int yy = y0; yy <= y1; yy++) {
                put_pixel(x, yy, SCOPE_COLOR);
            }
        }
        put_pixel(x, y, SCOPE_COLOR);
        prev_y = y;
    }
}

static bool parse_frame(char *line, gui_state_t *st)
{
    char *fields[16] = {0};
    int idx = 0;
    char *save = NULL;
    for (char *t = strtok_r(line, ",", &save); t && idx < 16; t = strtok_r(NULL, ",", &save)) {
        fields[idx++] = t;
    }
    if (idx < 15) return false;
    if (strcmp(fields[0], "AURA") != 0 || strcmp(fields[1], "1") != 0) return false;

    st->mode = atoi(fields[2]);
    st->track = atoi(fields[3]);
    st->instr = atoi(fields[4]);
    st->wave_pct = atoi(fields[5]);
    st->rec = atoi(fields[6]);
    st->play = atoi(fields[7]);
    st->loop_ms = atoi(fields[8]);
    st->head_ms = atoi(fields[9]);

    for (int tr = 0; tr < TL_TRACKS; tr++) {
        const char *s = fields[10 + tr];
        if ((int)strlen(s) < TL_BUCKETS) {
            return false;
        }
        for (int i = 0; i < TL_BUCKETS; i++) st->tl[tr][i] = hex_val(s[i]);
    }
    const char *sh = fields[14];
    if ((int)strlen(sh) < SCOPE_POINTS * 2) {
        return false;
    }
    for (int i = 0; i < SCOPE_POINTS; i++) {
        st->scope[i] = (uint8_t)((hex_val(sh[i * 2]) << 4) | hex_val(sh[i * 2 + 1]));
    }
    st->valid = true;
    return true;
}

static void render_gui_frame(const gui_state_t *st)
{
    memset(s_draw_buf, 0, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
    fill_rect(0, 0, LCD_H_RES, LCD_V_RES, BG_COLOR);

    /* Header */
    fill_rect(0, 0, LCD_H_RES, 40, 0x10A3);
    char line0[96];
    char line1[96];
    snprintf(line0, sizeof(line0), "M:%s  T:%d  I:%d  W:%d%%",
             mode_name_short(st->mode), st->track, st->instr, st->wave_pct);
    snprintf(line1, sizeof(line1), "R:%d P:%d L:%d H:%d",
             st->rec, st->play, st->loop_ms, st->head_ms);
    draw_text5x7(8, 6, line0, FG_COLOR, 2);
    draw_text5x7(8, 22, line1, FG_COLOR, 2);

    /* Scope panel */
    const int sx = 10, sy = 52, sw = 780, sh = 140;
    draw_rect(sx, sy, sw, sh, FRAME_COLOR);
    draw_synth_scope(st, sx, sy, sw, sh);

    /* Timeline panel */
    const int tx = 10, ty = 208, tw = 780, th = 260;
    draw_rect(tx, ty, tw, th, FRAME_COLOR);
    const int lane_h = 58;
    const int lane_w = 760;
    const int lane_x = tx + 10;
    const int bw = lane_w / TL_BUCKETS;
    for (int tr = 0; tr < TL_TRACKS; tr++) {
        int ly = ty + 8 + tr * (lane_h + 4);
        fill_rect(lane_x, ly, lane_w, lane_h, 0x10A3);
        for (int i = 0; i < TL_BUCKETS; i++) {
            uint16_t c = color_for_code(st->tl[tr][i]);
            int w = (st->tl[tr][i] == 1) ? 10 : 3;
            int x = lane_x + i * bw;
            fill_rect(x, ly + 5, w, lane_h - 10, c);
        }
    }

    int play_x = lane_x;
#if !DISPLAY_DIAGNOSTIC_LOCK
    if (st->loop_ms > 0) {
        play_x += (int)((int64_t)st->head_ms * lane_w / st->loop_ms);
    }
#endif
    if (play_x < lane_x) play_x = lane_x;
    if (play_x > lane_x + lane_w - 1) play_x = lane_x + lane_w - 1;
    fill_rect(play_x, ty + 5, 3, 250, PLAYHEAD_CLR);
}

static void uart_rx_task(void *arg)
{
    (void)arg;
    char line[1024];
    int l = 0;
    uint8_t ch;

    while (1) {
        int n = uart_read_bytes(GUI_UART_PORT, &ch, 1, pdMS_TO_TICKS(20));
        if (n <= 0) {
            continue;
        }

        if (ch == '\n') {
            line[l] = '\0';
            gui_state_t tmp = {0};
            if (parse_frame(line, &tmp)) {
                portENTER_CRITICAL(&s_state_lock);
                s_state = tmp;
                s_state_seq++;
                portEXIT_CRITICAL(&s_state_lock);
            }
            l = 0;
        } else if (ch != '\r') {
            if (l < (int)sizeof(line) - 1) {
                line[l++] = (char)ch;
            } else {
                l = 0;
            }
        }
    }
}

static void render_task(void *arg)
{
    (void)arg;
    gui_state_t snap = {0};
    uint32_t last_seq = 0;

    while (1) {
        portENTER_CRITICAL(&s_state_lock);
        snap = s_state;
        uint32_t seq = s_state_seq;
        portEXIT_CRITICAL(&s_state_lock);

        if (seq == last_seq) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        last_seq = seq;

#if !DISPLAY_DIAGNOSTIC_LOCK
        s_use_fb_a = !s_use_fb_a;
        s_draw_buf = s_use_fb_a ? s_fb_a : s_fb_b;
#endif

        if (!snap.valid) {
            /* Never push an empty/placeholder frame after lock; hold last good frame
             * to avoid occasional visible blink on malformed UART lines. */
            continue;
        }
        render_gui_frame(&snap);
#if !DISPLAY_DIAGNOSTIC_LOCK
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, LCD_H_RES, LCD_V_RES, s_draw_buf);
#endif
    }
}

bool crowpanel_gui_init(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << LCD_PIN_BL),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_cfg);
    gpio_set_level(LCD_PIN_BL, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_lcd_rgb_panel_config_t cfg = {
        .data_width    = 16,
#if DISPLAY_DIAGNOSTIC_LOCK
        .num_fbs       = 1,
#else
        .num_fbs       = 2,
#endif
        .clk_src       = LCD_CLK_SRC_DEFAULT,
        .disp_gpio_num = -1,
        .pclk_gpio_num  = LCD_PIN_PCLK,
        .vsync_gpio_num = LCD_PIN_VSYNC,
        .hsync_gpio_num = LCD_PIN_HSYNC,
        .de_gpio_num    = LCD_PIN_DE,
        .data_gpio_nums = LCD_DATA_PINS,
        .timings = {
            .pclk_hz           = 10 * 1000 * 1000,
            .h_res             = LCD_H_RES,
            .v_res             = LCD_V_RES,
            .hsync_pulse_width = 4,
            .hsync_back_porch  = 43,
            .hsync_front_porch = 8,
            .vsync_pulse_width = 4,
            .vsync_back_porch  = 12,
            .vsync_front_porch = 8,
            .flags.pclk_active_neg = true,
        },
        .psram_trans_align = 64,
        .flags.fb_in_psram = true,
    };

    if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) == 0) {
        ESP_LOGE(TAG, "No SPIRAM heap available; 800x480 RGB framebuffer needs PSRAM.");
        ESP_LOGE(TAG, "Enable SPIRAM in this project (sdkconfig defaults/menuconfig).");
        return false;
    }

    esp_err_t err = esp_lcd_new_rgb_panel(&cfg, &s_panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_rgb_panel failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Likely framebuffer allocation failure. Check SPIRAM config.");
        return false;
    }
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    /* RGB panel backlight is handled via GPIO; some IDF RGB panel drivers
     * return ESP_ERR_NOT_SUPPORTED for disp_on_off. Treat as optional. */
    esp_err_t disp_err = esp_lcd_panel_disp_on_off(s_panel, true);
    if (disp_err != ESP_OK && disp_err != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGE(TAG, "esp_lcd_panel_disp_on_off failed: %s", esp_err_to_name(disp_err));
        return false;
    }
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(
#if DISPLAY_DIAGNOSTIC_LOCK
        s_panel, 1, (void **)&s_fb_a));
    if (s_fb_a == NULL) {
        ESP_LOGE(TAG, "Failed to get RGB panel frame buffer");
        return false;
    }
#else
        s_panel, 2, (void **)&s_fb_a, (void **)&s_fb_b));
    if (s_fb_a == NULL || s_fb_b == NULL) {
        ESP_LOGE(TAG, "Failed to get RGB panel frame buffers");
        return false;
    }
#endif
    memset(s_fb_a, 0, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
#if !DISPLAY_DIAGNOSTIC_LOCK
    memset(s_fb_b, 0, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
#endif
    s_fb = s_fb_a;
    s_draw_buf = s_fb_a;
#if !DISPLAY_DIAGNOSTIC_LOCK
    s_use_fb_a = true;
#endif

    const uart_config_t uart_cfg = {
        .baud_rate = GUI_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GUI_UART_PORT, GUI_UART_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GUI_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GUI_UART_PORT, GUI_UART_TX_PIN, GUI_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    memset(&s_state, 0, sizeof(s_state));
    s_state_seq = 0;
    s_ready = true;
    ESP_LOGI(TAG, "CrowPanel GUI framebuffer renderer initialized");
    return true;
}

void crowpanel_gui_start(void)
{
    if (!s_ready) {
        return;
    }
    if (s_uart_task == NULL) {
        xTaskCreatePinnedToCore(uart_rx_task, "cp_uart_rx", 4096, NULL, 5, &s_uart_task, 0);
    }
    if (s_render_task == NULL) {
        xTaskCreatePinnedToCore(render_task, "cp_render", 6144, NULL, 4, &s_render_task, 1);
    }
}

