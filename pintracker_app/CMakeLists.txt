cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pintracker_app)

target_sources(app PRIVATE
    src/main.c
    src/dw3000_port.c

    src/dw3000/deca_interface.c
    src/dw3000/deca_compat.c
    src/dw3000/dw3000_device.c
)

target_include_directories(app PRIVATE
    src/dw3000
)