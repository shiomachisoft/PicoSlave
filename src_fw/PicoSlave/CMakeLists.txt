# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

project(PicoSlave C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

if (TARGET tinyusb_device)

    add_compile_options(-Wall
            -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
            -Wno-unused-function # we have some for the docs that aren't called
            -Wno-maybe-uninitialized
            )

    add_executable(PicoSlave
            main.c
            )

    # pull in common dependencies
    target_link_libraries(PicoSlave 
            pico_stdlib 
            hardware_spi 
            hardware_dma 
            hardware_i2c 
            pico_i2c_slave 
            hardware_pwm 
            pico_multicore
            hardware_sync
            hardware_clocks
            )

    # enable usb output, disable uart output
    pico_enable_stdio_usb(PicoSlave 1)
    pico_enable_stdio_uart(PicoSlave 0)

    # Run from RAM
    pico_set_binary_type(PicoSlave copy_to_ram)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(PicoSlave)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building PicoSlave because TinyUSB submodule is not initialized in the SDK")
endif()
