cmake_minimum_required(VERSION 3.20.2)
project(optimiluca)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
# Dependencies
find_package(Eigen3 REQUIRED)
# Libraries
add_subdirectory(src)

