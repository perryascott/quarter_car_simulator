cmake_minimum_required(VERSION 3.10)
project(quarter_car)

# Set C Standard
set(CMAKE_C_STANDARD 99)

# Include Directories
include_directories(include)

# Source Files
file(GLOB SOURCES "src/*.c")

# Add executable
add_executable(quarter_car ${SOURCES})