# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the path to our custom SDK config header before anything else
set(PICO_SDK_CONFIG_HEADER_PATH ${CMAKE_CURRENT_LIST_DIR}/pico_sdk_config.h)

set(PICO_BOARD pico CACHE STRING "Board type") # NOTE: Use "pico" or "pico_w"

# Disable TinyUSB (we use our own USB stack)
set(PICO_TINYUSB_PATH "" CACHE STRING "Disable TinyUSB")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(picousb C CXX ASM)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Executable name
add_executable(picousb src/main.c)

pico_set_program_name(picousb "picousb")
pico_set_program_version(picousb "0.3")

target_sources(picousb PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c
    ${CMAKE_CURRENT_LIST_DIR}/src/helpers.c
    ${CMAKE_CURRENT_LIST_DIR}/src/picousb.c
    ${CMAKE_CURRENT_LIST_DIR}/src/ring.c

    # NOTE: Drivers automatically register, comment out if unused
    ${CMAKE_CURRENT_LIST_DIR}/src/drivers/astm/astm.c
)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(picousb 1)
pico_enable_stdio_usb(picousb 0)  # Ensure USB stdio is disabled

# Add the standard include files to the build
target_include_directories(picousb PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/src
)

# Add any user requested libraries
target_link_libraries(picousb
  # pico_cyw43_arch_none # NOTE: Enable this for "pico_w"
    pico_stdlib
    hardware_resets
    hardware_irq
)

pico_add_extra_outputs(picousb)
