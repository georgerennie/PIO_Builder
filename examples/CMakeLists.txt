cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)
project(examples)
pico_sdk_init()

set(CXX_STANDARD 14)

add_executable(pio_blink pio_blink.cpp)
target_include_directories(pio_blink PRIVATE ../)
target_link_libraries(pio_blink pico_stdlib hardware_pio)

add_executable(variable_delay variable_delay.cpp)
target_include_directories(variable_delay PRIVATE ../)
target_link_libraries(variable_delay pico_stdlib hardware_pio)

add_executable(hello_pio hello_pio.cpp)
target_include_directories(hello_pio PRIVATE ../)
target_link_libraries(hello_pio pico_stdlib hardware_pio)

pico_add_extra_outputs(pio_blink)
pico_add_extra_outputs(variable_delay)
pico_add_extra_outputs(hello_pio)
