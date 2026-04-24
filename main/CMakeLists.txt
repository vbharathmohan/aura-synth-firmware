idf_component_register(
    SRCS
        # --- Shared (everyone) ---
        "main.c"
        "shared_state.c"
 
        # --- Audio pipeline (Batch 1: foundation) ---
        "audio_block.c"
        "i2s_output.c"
 
        # --- Audio pipeline (Batch 2: sources) ---
        "synth_voice.c"
        "sampler.c"
 
        # --- Audio pipeline (Batch 3: pipeline) ---
        "effects.c"
        "mixer.c"
        "loop_recorder.c"
        "audio_task.c"
 
        # --- Hardware (teammate) ---
        # "sensors.c"
        # "controls.c"
        # "leds.c"
        # "lcd_comm.c"
        # "gesture.c"
        # "control_task.c"
 
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_driver_i2s
        esp_driver_gpio
        esp_common
)
 
# --- Embed WAV samples into flash ---
# Each file becomes accessible as:
#   extern const uint8_t name_wav_start[] asm("_binary_name_wav_start");
#   extern const uint8_t name_wav_end[]   asm("_binary_name_wav_end");
#
# Add your samples here:
target_add_binary_data(${COMPONENT_LIB} "../samples/kick.wav"    BINARY)
target_add_binary_data(${COMPONENT_LIB} "../samples/snare.wav"   BINARY)
target_add_binary_data(${COMPONENT_LIB} "../samples/hihat.wav"   BINARY)