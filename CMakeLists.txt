# Set minimum version of CMake
cmake_minimum_required(VERSION 3.25)

# Include build functions from pico sdk
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(csc2003-b1 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# create a pico-sdk subdirectory in our projects for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME}
    main.c
        )

# Link to pico_stdlib (gpio, time, etc)
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_pwm hardware_timer hardware_uart hardware_gpio)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(${PROJECT_NAME})

# FOR DEBUGGING - Enable USB output, disable uart output
# Open serial terminal to view output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
