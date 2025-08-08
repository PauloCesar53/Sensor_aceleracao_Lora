
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(lora_rx_01 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(lora_rx_01 
    lora_rx_01.c
    lib/ssd1306.c 
        )

pico_set_program_name(lora_rx_01 "lora_rx_01")
pico_set_program_version(lora_rx_01 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(lora_rx_01 0)
pico_enable_stdio_usb(lora_rx_01 1)

# Add the standard library to the build
target_link_libraries(lora_rx_01
    pico_stdlib
    hardware_i2c)

# Add the standard include files to the build
target_include_directories(lora_rx_01 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(lora_rx_01 
        hardware_spi
        )

pico_add_extra_outputs(lora_rx_01)

