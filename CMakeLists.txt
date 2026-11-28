cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Spinal_tester)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

option(SPINAL_UART_SMOKE "Build the standalone dual-UART text test" OFF)
option(SPINAL_I2C_DIAG "Build the standalone I2C diagnostic" OFF)
if(SPINAL_UART_SMOKE AND SPINAL_I2C_DIAG)
    message(FATAL_ERROR "Select only one standalone diagnostic")
endif()
if(SPINAL_UART_SMOKE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SPINAL_UART_SMOKE=1)
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME Spinal_tester_uart_smoke)
endif()

if(SPINAL_I2C_DIAG)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SPINAL_I2C_DIAG=1)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE App/Src/i2c_diagnostic.c)
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME Spinal_tester_i2c_diag)
endif()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/Src/app_bootstrap.c
    BSP/Src/bsp_calibration.c
    BSP/Src/bsp_i2c.c
    BSP/Src/bsp_time.c
    BSP/Src/bsp_uart.c
    Devices/Src/bno055.c
    Devices/Src/tca9548a.c
    Services/Src/diagnostics_service.c
    Services/Src/neck_motion_algorithm.c
    Services/Src/protocol_service.c
    Services/Src/sensor_service.c
    Tasks/Src/acquisition_task.c
    Tasks/Src/command_task.c
    Tasks/Src/communication_task.c
    Tasks/Src/health_task.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App/Inc
    BSP/Inc
    Common/Inc
    Config/Inc
    Devices/Inc
    Services/Inc
    Tasks/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
