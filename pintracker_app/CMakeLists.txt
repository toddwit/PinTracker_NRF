cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pintracker_app)

set(PINTRACKER_ROLE "tag" CACHE STRING "PinTracker role: tag or anchor")
set(PINTRACKER_ANCHOR_ID "0" CACHE STRING "Anchor ID used when PINTRACKER_ROLE=anchor")

target_sources(app PRIVATE
    src/main.c
    src/dw3000_port.c
	src/port.c

    src/dw3000/deca_interface.c
    src/dw3000/deca_compat.c
    src/dw3000/dw3000_device.c
    src/dw3000/deca_probe_interface.c
)

target_include_directories(app PRIVATE
	src
	src/dw3000
)

if(PINTRACKER_ROLE STREQUAL "anchor")
	target_compile_definitions(app PRIVATE
		PINTRACKER_ROLE_ANCHOR=1
		PINTRACKER_ANCHOR_ID=${PINTRACKER_ANCHOR_ID}
	)
else()
	target_compile_definitions(app PRIVATE PINTRACKER_ROLE_TAG=1)
endif()
