# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/Users/shuichi/pico/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(sega-dsb-clone-sq C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(sega-dsb-clone-sq
  main.cpp
  hw_config.cpp
  )

pico_generate_pio_header(sega-dsb-clone-sq ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)

pico_set_program_name(sega-dsb-clone-sq "sega-dsb-clone-sq")
pico_set_program_version(sega-dsb-clone-sq "0.1")

pico_enable_stdio_uart(sega-dsb-clone-sq 1)
pico_enable_stdio_usb(sega-dsb-clone-sq 0)

# Add the standard library to the build
target_link_libraries(sega-dsb-clone-sq
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(sega-dsb-clone-sq PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

target_compile_options(sega-dsb-clone-sq PRIVATE
  -Wall -Wextra -Werror
  -Wno-missing-field-initializers
)

add_subdirectory(thirdparty/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src)
add_subdirectory(thirdparty/picomp3lib/src)

target_compile_options(picomp3lib INTERFACE 
  -Wno-unused-parameter
 )

# Add any user requested libraries
target_link_libraries(sega-dsb-clone-sq
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_clocks
        no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
        picomp3lib
        )

pico_add_extra_outputs(sega-dsb-clone-sq)

