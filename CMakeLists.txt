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
set(CMAKE_PROJECT_NAME Sx1262_lorawan_stm32)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Find all .c files in sx126x_driver/src
file(GLOB SX126X_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../sx126x_driver/src/*.c"
)
# Find all .c files in mac folder
file(GLOB_RECURSE  MAC_SOURCES "/home/wyld_linux/Documents/lora_net/LoRaMac-node/src/mac/*.c")

# Find all .h files in mac folder
file(GLOB_RECURSE  MAC_HEADERS "/home/wyld_linux/Documents/lora_net/LoRaMac-node/src/mac/*.h")


# Find all .c files in soft-se folder
file(GLOB_RECURSE SOFT_SE_SOURCE "/home/wyld_linux/Documents/lora_net/LoRaMac-node/src/peripherals/*.c")

# Find all .h files in soft-se  folder
file(GLOB_RECURSE  SOFT_SE_HEADERS "/home/wyld_linux/Documents/lora_net/LoRaMac-node/src/peripherals/*.h")

# Find all .c files in Loramac folder
file(GLOB_RECURSE LORAMAC_SOURCE "/home/wyld_linux/Documents/lora_net/LoRaMac-node/src/apps/*.c")


# Find all .h files in Loramac folder
file(GLOB_RECURSE LORAMAC_HEADERS "/home/wyld_linux/Documents/lora_net/LoRaMac-node/src/apps/*.h")

# Find all .h files in radio folder
file(GLOB_RECURSE RADIO_HEADERS "/home/wyld_linux/Documents/lora_net/LoRaMac-node/src/radio/*.h")

# Find all .c files in the Radio folder
file(GLOB_RECURSE RADIO_SOURCES "/home/wyld_linux/Documents/lora_net/LoRaMac-node/src/radio/*.c")

# Extract unique include directories
set(INCLUDE_DIRS "")

foreach(header ${MAC_HEADERS})
    get_filename_component(HEADER_DIR ${header} DIRECTORY)
    list(APPEND INCLUDE_DIRS ${HEADER_DIR})
endforeach()

foreach(header ${SOFT_SE_HEADERS})
    get_filename_component(HEADER_DIR ${header} DIRECTORY)
    list(APPEND INCLUDE_DIRS ${HEADER_DIR})
endforeach()

foreach(header ${LORAMAC_HEADERS})
    get_filename_component(HEADER_DIR ${header} DIRECTORY)
    list(APPEND INCLUDE_DIRS ${HEADER_DIR})
endforeach()
foreach(header ${RADIO_HEADERS})
    get_filename_component(HEADER_DIR ${header} DIRECTORY)
    list(APPEND INCLUDE_DIRS ${HEADER_DIR})
endforeach()

list(REMOVE_DUPLICATES INCLUDE_DIRS)  # Remove duplicate directories


# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${SX126X_SOURCES}
    /home/wyld_linux/Documents/lora_net/LoRaMac-node/src/boards/mcu/utilities.c
    ${MAC_SOURCES}
    ${SOFT_SE_SOURCE}
    ${LORAMAC_SOURCE}
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/../sx126x_driver/src
    /home/wyld_linux/Documents/lora_net/LoRaMac-node/src/boards
    ${INCLUDE_DIRS}  # Corrected to include all header directories
    /home/wyld_linux/Documents/lora_net/LoRaMac-node/src/apps/LoRaMac/common

    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
