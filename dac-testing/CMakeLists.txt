cmake_minimum_required(VERSION 3.13)

# VS Code Pico extension typically sets PICO_SDK_PATH automatically.
include(pico_sdk_import.cmake)

project(pcm5102a_tone_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pcm5102a_tone_test dac_tone_test.c)
pico_generate_pio_header(pcm5102a_tone_test ${CMAKE_CURRENT_LIST_DIR}/dac_tone_test.pio)
target_link_libraries(pcm5102a_tone_test pico_stdlib hardware_pio hardware_clocks m)
pico_enable_stdio_usb(pcm5102a_tone_test 1)
pico_enable_stdio_uart(pcm5102a_tone_test 0)
pico_add_extra_outputs(pcm5102a_tone_test)
