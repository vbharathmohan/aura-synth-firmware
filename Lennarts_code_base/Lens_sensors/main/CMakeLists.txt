idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES vl53l0x esp_driver_dac driver esp_driver_uart esp_driver_i2s esp_timer esp_driver_gpio esp_driver_i2c)

target_add_binary_data(__idf_main "../song.wav" BINARY)
target_add_binary_data(__idf_main "../song_fixed_new.wav" BINARY)