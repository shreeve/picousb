# ==============================================================================
# Generated Cmake Pico project file
# ==============================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# NOTE: Do not modify this section, it's needed for the VS Code Extension

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)

if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# ==============================================================================

set(PICO_BOARD pico CACHE STRING "Board type") # NOTE: Use "pico" or "pico_w"

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(picousb C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Executable name
add_executable(picousb picousb.c )

pico_set_program_name(picousb "picousb")
pico_set_program_version(picousb "0.3")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(picousb 1)
pico_enable_stdio_usb(picousb 0)

# Add the standard library to the build
target_link_libraries(picousb
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(picousb PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/..
)

# Add any user requested libraries
target_link_libraries(picousb
  # pico_cyw43_arch_none # NOTE: Enable this for "pico_w"
	pico_stdlib
	hardware_resets
	hardware_irq
)

pico_add_extra_outputs(picousb)
