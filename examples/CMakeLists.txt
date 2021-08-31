cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)
project(examples)
pico_sdk_init()

set(CXX_STANDARD 14)


add_executable(pio_blinky
	pio_blinky.cpp
)

target_include_directories(pio_blinky PRIVATE ../)

target_link_libraries(pio_blinky
	pico_stdlib
	hardware_pio
)

pico_add_extra_outputs(pio_blinky)
