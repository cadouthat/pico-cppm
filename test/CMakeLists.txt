cmake_minimum_required(VERSION 3.13)

include(../pico_sdk_import.cmake)

add_subdirectory(.. build)

project(pico_cppm_tests C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(cppm_decoder_test
  cppm_decoder_test.cpp
)
pico_enable_stdio_usb(cppm_decoder_test 1)
target_link_libraries(cppm_decoder_test
  pico_stdlib
  pico_cppm
)
pico_add_extra_outputs(cppm_decoder_test)

add_executable(cppm_encoder_decoder_test
  cppm_encoder_decoder_test.cpp
)
pico_enable_stdio_usb(cppm_encoder_decoder_test 1)
target_link_libraries(cppm_encoder_decoder_test
  pico_stdlib
  pico_cppm
)
pico_add_extra_outputs(cppm_encoder_decoder_test)
