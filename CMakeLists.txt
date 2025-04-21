# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define o nome do projeto como RaveController
set(PROJECTNAME "RaveController")

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(${PROJECTNAME} C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(${PROJECTNAME} ${PROJECTNAME}.c src/ssd1306.c src/np_led.c)

pico_set_program_name(${PROJECTNAME} "${PROJECTNAME}")
pico_set_program_version(${PROJECTNAME} "0.1")


# Generate PIO header
pico_generate_pio_header(${PROJECTNAME} ${CMAKE_CURRENT_LIST_DIR}/ws2818b.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${PROJECTNAME} 1)
pico_enable_stdio_usb(${PROJECTNAME} 0)

# Add the standard library to the build
target_link_libraries(${PROJECTNAME}
        pico_stdlib
)

# Add the standard include files to the build
target_include_directories(${PROJECTNAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(${PROJECTNAME} 
    hardware_i2c
    hardware_pio
    hardware_clocks
    hardware_timer
    hardware_adc
    hardware_pwm
)

pico_add_extra_outputs(${PROJECTNAME})

