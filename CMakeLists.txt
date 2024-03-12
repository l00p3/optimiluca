cmake_minimum_required(VERSION 3.20.2)
project(optimiluca)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Set output bin folder
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Dependencies
find_package(Eigen3 REQUIRED)

# Subdirs
add_subdirectory(src)

