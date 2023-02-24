cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_cppm C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_library(pico_cppm
  src/cppm_decoder.cpp)

target_include_directories(pico_cppm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(pico_cppm PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src )

target_link_libraries(pico_cppm
  pico_stdlib
  hardware_dma
  hardware_pio
)

pico_generate_pio_header(pico_cppm ${CMAKE_CURRENT_LIST_DIR}/src/pio/cppm_decoder.pio)
