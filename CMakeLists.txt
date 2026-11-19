# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ADCtesting C CXX ASM)
pico_sdk_init()

add_executable(ADCtesting ADCtesting.c)

pico_enable_stdio_uart(ADCtesting 0)
pico_enable_stdio_usb(ADCtesting 1)

pico_generate_pio_header(ADCtesting
    "${CMAKE_CURRENT_LIST_DIR}/pcm1808_rx.pio"
)

target_link_libraries(ADCtesting PRIVATE
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_clocks
)

pico_add_extra_outputs(ADCtesting)