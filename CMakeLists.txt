cmake_minimum_required(VERSION 3.20.2)
project(optimiluca)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_BUILD_TYPE Release)

# Set output bin folder
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

# Subdirs
add_subdirectory(src)

