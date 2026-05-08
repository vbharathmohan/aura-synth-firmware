idf_component_register(
    SRCS
        "main.c"
        "shared_state.c"
        "audio_block.c"
        "i2s_output.c"
        "synth_voice.c"
        "sampler.c"
        "effects.c"
        "mixer.c"
        "loop_recorder.c"
        "audio_task.c"
        "audio_scope.c"
        "telemetry_uart.c"
        "sensor_task.cpp"
        "led_task.cpp"
        "panel_input.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_driver_i2s
        esp_driver_i2c
        esp_driver_gpio
        esp_driver_uart
        esp_adc
        led_strip
        esp_timer
        esp_common
        vl53l0x
)

# --- Embed WAV samples into flash ---
# Each file becomes accessible as:
#   extern const uint8_t name_wav_start[] asm("_binary_name_wav_start");
#   extern const uint8_t name_wav_end[]   asm("_binary_name_wav_end");
#
# Slot indices live in shared_state.h (SAMPLE_SLOT_*). Keep these
# lines and the sampler_register() calls in main.c in sync.

# Drum kit (kick / snare / hihat trigger directly from the pad
# buttons in INTEGRATION_MODE; the ToF array does not play these).
target_add_binary_data(${COMPONENT_LIB} "../samples/kick.wav"           BINARY)
target_add_binary_data(${COMPONENT_LIB} "../samples/snare.wav"          BINARY)
target_add_binary_data(${COMPONENT_LIB} "../samples/hihat.wav"          BINARY)
target_add_binary_data(${COMPONENT_LIB} "../samples/clap.wav"          BINARY)

# Melodic instruments (ToF array plays these in C-major).
target_add_binary_data(${COMPONENT_LIB} "../samples/piano_c4.wav"       BINARY)
target_add_binary_data(${COMPONENT_LIB} "../samples/steel_drum_c4.wav"  BINARY)
target_add_binary_data(${COMPONENT_LIB} "../samples/trumpet_c6.wav"     BINARY)
target_add_binary_data(${COMPONENT_LIB} "../samples/808_c1.wav"         BINARY)
