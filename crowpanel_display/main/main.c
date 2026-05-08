#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "crowpanel_gui.h"

static const char *TAG = "crowpanel_main";

void app_main(void)
{
    ESP_LOGI(TAG, "CrowPanel display firmware (RX + GUI only)");
    if (!crowpanel_gui_init()) {
        ESP_LOGE(TAG, "crowpanel_gui_init failed");
        return;
    }
    crowpanel_gui_start();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

