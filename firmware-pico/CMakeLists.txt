cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(bms C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(bms)
pico_generate_pio_header(bms ${CMAKE_CURRENT_LIST_DIR}/bms.pio)
target_sources(bms PRIVATE bms.c)
target_link_libraries(bms PRIVATE pico_stdlib hardware_pio hardware_spi hardware_sleep)

#pico_enable_stdio_usb(bms 1)
#pico_enable_stdio_uart(bms 0)

pico_add_extra_outputs(bms)
