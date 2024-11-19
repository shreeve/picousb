# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
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
cmake_minimum_required(VERSION 3.20)

# disable TinyUSB: Comment out in ${PICO_SDK_PATH}/src/rp2_common/CMakeLists.txt
#
#   # pico_add_subdirectory(tinyusb)
#   # pico_add_subdirectory(pico_stdio_usb)

# pull in standard Pico SDK settings
include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")

# set project name to the name of this folder
cmake_path(GET CMAKE_CURRENT_SOURCE_DIR FILENAME project_name)

project(${project_name})

pico_sdk_init()

add_executable(${project_name}
  host.c
)

target_link_libraries(${project_name} PRIVATE
  pico_stdlib
  hardware_resets
  hardware_irq
)

# uncomment to disable stdio via uart
# pico_enable_stdio_uart(${project_name} 0)

# uncomment to create uf2 files
# pico_add_extra_outputs(${project_name})
