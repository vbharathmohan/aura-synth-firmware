idf_component_register(
    SRCS
        "main.c"
        "crowpanel_gui.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_driver_uart
        esp_driver_gpio
        esp_lcd
        esp_lvgl_port
        esp_common
)

