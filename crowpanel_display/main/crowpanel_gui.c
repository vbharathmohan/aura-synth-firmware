#include "crowpanel_gui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

static const char *TAG = "crowpanel_gui";

#define LCD_H_RES 800
#define LCD_V_RES 480

/* Elecrow 5.0 RGB pins */
#define PIN_LCD_D0   8
#define PIN_LCD_D1   3
#define PIN_LCD_D2   46
#define PIN_LCD_D3   9
#define PIN_LCD_D4   1
#define PIN_LCD_D5   5
#define PIN_LCD_D6   6
#define PIN_LCD_D7   7
#define PIN_LCD_D8   15
#define PIN_LCD_D9   16
#define PIN_LCD_D10  4
#define PIN_LCD_D11  45
#define PIN_LCD_D12  48
#define PIN_LCD_D13  47
#define PIN_LCD_D14  21
#define PIN_LCD_D15  14
#define PIN_LCD_DE   40
#define PIN_LCD_VSYNC 41
#define PIN_LCD_HSYNC 39
#define PIN_LCD_PCLK  0
#define PIN_LCD_BL    2

/* RX-only telemetry input from instrument */
#define GUI_UART_PORT   UART_NUM_1
#define GUI_UART_RX_PIN GPIO_NUM_44
#define GUI_UART_BAUD   921600

#define TL_BUCKETS    64
#define TL_TRACKS     4
#define SCOPE_POINTS  64

typedef struct {
    int mode, track, instr, wave_pct, rec, play, loop_ms, head_ms;
    uint8_t tl[TL_TRACKS][TL_BUCKETS];
    uint8_t scope[SCOPE_POINTS];
    bool valid;
} gui_state_t;

static gui_state_t s_g;
static TaskHandle_t s_task;
static bool s_ready;

static lv_obj_t *s_lbl_mode, *s_lbl_track, *s_lbl_instr, *s_lbl_wave, *s_lbl_flags;
static lv_obj_t *s_scope;
static lv_obj_t *s_lane[TL_TRACKS][TL_BUCKETS];
static lv_obj_t *s_playhead;

static uint8_t hex_val(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0;
}

static lv_color_t color_for_code(uint8_t code)
{
    switch (code) {
    case 1: return lv_color_hex(0x66f2d5); /* synth */
    case 2: return lv_color_hex(0xa78bfa); /* piano */
    case 3: return lv_color_hex(0x60a5fa); /* steel */
    case 4: return lv_color_hex(0xf59e0b); /* trumpet */
    case 5: return lv_color_hex(0xfb7185); /* 808 */
    case 6: return lv_color_hex(0xff4d6d); /* kick */
    case 7: return lv_color_hex(0xf97316); /* snare */
    case 8: return lv_color_hex(0xfde047); /* hihat */
    case 9: return lv_color_hex(0x34d399); /* clap */
    default: return lv_color_hex(0x2a355f);
    }
}

static const char *mode_name_ui(int m)
{
    switch (m) {
    case 0: return "SYNTH";
    case 1: return "DRUMS";
    case 2: return "PIANO";
    case 3: return "FX";
    case 4: return "SAMPLE";
    default: return "?";
    }
}

static const char *instr_name_ui(int i)
{
    switch (i) {
    case 0: return "PIANO";
    case 1: return "STEEL";
    case 2: return "TRUMPET";
    case 3: return "808";
    default: return "?";
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
        for (int i = 0; i < TL_BUCKETS; i++) st->tl[tr][i] = hex_val(s[i]);
    }
    const char *sh = fields[14];
    for (int i = 0; i < SCOPE_POINTS; i++) {
        st->scope[i] = (uint8_t)((hex_val(sh[i * 2]) << 4) | hex_val(sh[i * 2 + 1]));
    }

    st->valid = true;
    return true;
}

static void ui_build(void)
{
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x0b1020), 0);
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xe6e9ff), 0);

    lv_obj_t *hdr = lv_obj_create(lv_scr_act());
    lv_obj_set_size(hdr, LCD_H_RES, 56);
    lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_bg_color(hdr, lv_color_hex(0x111a33), 0);

    s_lbl_mode = lv_label_create(hdr);
    s_lbl_track = lv_label_create(hdr);
    s_lbl_instr = lv_label_create(hdr);
    s_lbl_wave = lv_label_create(hdr);
    s_lbl_flags = lv_label_create(hdr);
    lv_obj_align(s_lbl_mode, LV_ALIGN_LEFT_MID, 8, -12);
    lv_obj_align(s_lbl_track, LV_ALIGN_LEFT_MID, 180, -12);
    lv_obj_align(s_lbl_instr, LV_ALIGN_LEFT_MID, 320, -12);
    lv_obj_align(s_lbl_wave, LV_ALIGN_LEFT_MID, 500, -12);
    lv_obj_align(s_lbl_flags, LV_ALIGN_LEFT_MID, 8, 12);

    s_scope = lv_chart_create(lv_scr_act());
    lv_obj_set_size(s_scope, 780, 140);
    lv_obj_align(s_scope, LV_ALIGN_TOP_MID, 0, 64);
    lv_chart_set_type(s_scope, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(s_scope, SCOPE_POINTS);
    lv_chart_set_range(s_scope, LV_CHART_AXIS_PRIMARY_Y, 0, 255);
    lv_chart_add_series(s_scope, lv_color_hex(0x66f2d5), LV_CHART_AXIS_PRIMARY_Y);

    lv_obj_t *tl = lv_obj_create(lv_scr_act());
    lv_obj_set_size(tl, 780, 260);
    lv_obj_align(tl, LV_ALIGN_TOP_MID, 0, 212);
    lv_obj_set_style_bg_color(tl, lv_color_hex(0x0e1630), 0);
    lv_obj_set_style_border_color(tl, lv_color_hex(0x223060), 0);
    lv_obj_set_style_pad_all(tl, 8, 0);

    int lane_h = 58;
    int lane_w = 760;
    int bw = lane_w / TL_BUCKETS;
    if (bw < 2) bw = 2;

    for (int tr = 0; tr < TL_TRACKS; tr++) {
        lv_obj_t *lane = lv_obj_create(tl);
        lv_obj_set_size(lane, lane_w, lane_h);
        lv_obj_align(lane, LV_ALIGN_TOP_LEFT, 10, 8 + tr * (lane_h + 4));
        lv_obj_set_style_bg_color(lane, lv_color_hex(0x111a33), 0);
        lv_obj_set_style_border_width(lane, 0, 0);
        lv_obj_set_style_radius(lane, 8, 0);
        lv_obj_set_style_pad_all(lane, 0, 0);

        for (int i = 0; i < TL_BUCKETS; i++) {
            lv_obj_t *b = lv_obj_create(lane);
            lv_obj_set_size(b, bw, lane_h - 10);
            lv_obj_align(b, LV_ALIGN_TOP_LEFT, i * bw, 5);
            lv_obj_set_style_bg_color(b, lv_color_hex(0x2a355f), 0);
            lv_obj_set_style_border_width(b, 0, 0);
            lv_obj_set_style_radius(b, 2, 0);
            s_lane[tr][i] = b;
        }
    }

    s_playhead = lv_obj_create(tl);
    lv_obj_set_size(s_playhead, 3, 250);
    lv_obj_align(s_playhead, LV_ALIGN_TOP_LEFT, 10, 5);
    lv_obj_set_style_bg_color(s_playhead, lv_color_hex(0xff4d6d), 0);
    lv_obj_set_style_border_width(s_playhead, 0, 0);
}

static void ui_apply(const gui_state_t *st)
{
    if (!st->valid) return;
    char buf[96];
    snprintf(buf, sizeof(buf), "MODE: %s", mode_name_ui(st->mode));
    lv_label_set_text(s_lbl_mode, buf);
    snprintf(buf, sizeof(buf), "TRACK: %d", st->track);
    lv_label_set_text(s_lbl_track, buf);
    snprintf(buf, sizeof(buf), "INSTR: %s", instr_name_ui(st->instr));
    lv_label_set_text(s_lbl_instr, buf);
    snprintf(buf, sizeof(buf), "WAVE: %d%%", st->wave_pct);
    lv_label_set_text(s_lbl_wave, buf);
    snprintf(buf, sizeof(buf), "REC:%d PLAY:%d LOOP:%dms HEAD:%dms", st->rec, st->play, st->loop_ms, st->head_ms);
    lv_label_set_text(s_lbl_flags, buf);

    lv_chart_series_t *ser = lv_chart_get_series_next(s_scope, NULL);
    if (ser) {
        for (int i = 0; i < SCOPE_POINTS; i++) {
            lv_chart_set_value_by_id(s_scope, ser, (uint32_t)i, (int32_t)st->scope[i]);
        }
        lv_chart_refresh(s_scope);
    }

    for (int tr = 0; tr < TL_TRACKS; tr++) {
        for (int i = 0; i < TL_BUCKETS; i++) {
            uint8_t code = st->tl[tr][i];
            lv_obj_t *b = s_lane[tr][i];
            lv_obj_set_style_bg_color(b, color_for_code(code), 0);
            lv_obj_set_width(b, (code == 1) ? 11 : 3);
            lv_obj_set_style_radius(b, (code == 1) ? 5 : 2, 0);
        }
    }

    int x = 10;
    if (st->loop_ms > 0) {
        x += (int)((int64_t)st->head_ms * 760 / st->loop_ms);
        if (x < 10) x = 10;
        if (x > 770) x = 770;
    }
    lv_obj_set_x(s_playhead, x);
}

static void uart_init_rx_only(void)
{
    const uart_config_t cfg = {
        .baud_rate = GUI_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(GUI_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GUI_UART_PORT, UART_PIN_NO_CHANGE, GUI_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GUI_UART_PORT, 2048, 0, 0, NULL, 0));
}

static void gui_rx_task(void *param)
{
    (void)param;
    char line[1024];
    int l = 0;
    uint8_t ch;

    while (1) {
        int n = uart_read_bytes(GUI_UART_PORT, &ch, 1, pdMS_TO_TICKS(10));
        if (n > 0) {
            if (ch == '\n') {
                line[l] = '\0';
                gui_state_t tmp = {0};
                if (parse_frame(line, &tmp) && lvgl_port_lock(0)) {
                    s_g = tmp;
                    ui_apply(&s_g);
                    lvgl_port_unlock();
                }
                l = 0;
            } else if (ch != '\r') {
                if (l < (int)sizeof(line) - 1) line[l++] = (char)ch;
                else l = 0;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

bool crowpanel_gui_init(void)
{
    gpio_config_t bl = {};
    bl.pin_bit_mask = (1ULL << PIN_LCD_BL);
    bl.mode = GPIO_MODE_OUTPUT;
    gpio_config(&bl);
    gpio_set_level(PIN_LCD_BL, 1);

    esp_lcd_rgb_panel_config_t rgb_cfg = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .timings = {
            .pclk_hz = 15000000,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            .hsync_pulse_width = 4,
            .hsync_back_porch = 43,
            .hsync_front_porch = 8,
            .vsync_pulse_width = 4,
            .vsync_back_porch = 12,
            .vsync_front_porch = 8,
            .flags = {.pclk_active_neg = 1},
        },
        .data_width = 16,
        .psram_trans_align = 64,
        .hsync_gpio_num = PIN_LCD_HSYNC,
        .vsync_gpio_num = PIN_LCD_VSYNC,
        .de_gpio_num = PIN_LCD_DE,
        .pclk_gpio_num = PIN_LCD_PCLK,
        .disp_gpio_num = -1,
        .data_gpio_nums = {
            PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3,
            PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7,
            PIN_LCD_D8, PIN_LCD_D9, PIN_LCD_D10, PIN_LCD_D11,
            PIN_LCD_D12, PIN_LCD_D13, PIN_LCD_D14, PIN_LCD_D15,
        },
        .flags = {.fb_in_psram = 1},
    };

    esp_lcd_panel_handle_t panel = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&rgb_cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .panel_handle = panel,
        .buffer_size = LCD_H_RES * 60,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .rotation = {.swap_xy = false, .mirror_x = false, .mirror_y = false},
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {.buff_dma = true, .buff_spiram = true, .swap_bytes = false},
    };
    lvgl_port_add_disp(&disp_cfg);

    uart_init_rx_only();

    if (lvgl_port_lock(0)) {
        ui_build();
        lvgl_port_unlock();
    }

    s_ready = true;
    ESP_LOGI(TAG, "CrowPanel GUI RX-only initialized");
    return true;
}

void crowpanel_gui_start(void)
{
    if (!s_ready || s_task) return;
    xTaskCreatePinnedToCore(gui_rx_task, "crowpanel_gui", 8192, NULL, 3, &s_task, 0);
}

