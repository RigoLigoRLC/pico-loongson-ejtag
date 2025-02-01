
cmake_minimum_required(VERSION 3.21)
set(PICO_TOOLCHAIN_PATH $ENV{PICO_SDK_COMPILER_PATH})
include($ENV{PICO_SDK_BASE}/external/pico_sdk_import.cmake)

project(pico-loongson-ejtag LANGUAGES C CXX ASM)
# set(PICO_DEOPTIMIZED_DEBUG 1)  # Uncomment under Debug mode to disable -Og flag
pico_sdk_init()

file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/*.c)

add_compile_options(
    -Wall -Wextra -Werror=uninitialized -Werror=return-type -Werror=unused-result
)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/inc
    ${CMAKE_CURRENT_LIST_DIR}/inc/letter-shell
)
add_executable(${PROJECT_NAME}
    ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    hardware_pio
    hardware_dma
    pico_stdlib
    pico_unique_id
    tinyusb_device
    tinyusb_board
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/ejtag.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/ws2812.pio)

pico_add_extra_outputs(${PROJECT_NAME})
