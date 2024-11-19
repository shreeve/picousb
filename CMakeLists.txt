# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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
# ====================================================================================

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico CACHE STRING "Board type") # NOTE: Use "pico" or "pico_w"

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(piccolo C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Executable name
add_executable(piccolo piccolo.c)

pico_set_program_name(piccolo "piccolo")
pico_set_program_version(piccolo "0.3")

target_sources(piccolo PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/piccolo.c
        ${CMAKE_CURRENT_LIST_DIR}/picousb.c
        ${CMAKE_CURRENT_LIST_DIR}/helpers.c
)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(piccolo 1)
pico_enable_stdio_usb(piccolo 0)

# Add the standard library to the build
target_link_libraries(piccolo
        pico_stdlib
)

# Add the standard include files to the build
target_include_directories(piccolo PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/..
)

# Add any user requested libraries
target_link_libraries(piccolo
  # pico_cyw43_arch_none # NOTE: Enable this for "pico_w"
	pico_stdlib
	hardware_resets
	hardware_irq
)

pico_add_extra_outputs(piccolo)
