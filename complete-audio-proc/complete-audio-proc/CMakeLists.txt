cmake_minimum_required(VERSION 3.13)

# PICO_SDK_PATH must be set, as when using the Raspberry Pi Pico VS Code extension.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(pico2w_record_audio C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(pico2w_record_audio main.c)
pico_generate_pio_header(pico2w_record_audio ${CMAKE_CURRENT_LIST_DIR}/audio_full_duplex.pio)
target_link_libraries(pico2w_record_audio
    pico_stdlib
    hardware_pio
    hardware_clocks
)
pico_enable_stdio_usb(pico2w_record_audio 1)
pico_enable_stdio_uart(pico2w_record_audio 0)
pico_add_extra_outputs(pico2w_record_audio)
