add_library(libpiccolo STATIC
    src/ring.c
    src/picousb.c
    src/helpers.c
)

target_include_directories(libpiccolo PUBLIC
    inc
)

target_link_libraries(libpiccolo PUBLIC
    pico_stdlib
    pico_cyw43_arch_none
    hardware_resets
    hardware_irq
)
