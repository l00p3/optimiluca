cmake_minimum_required(VERSION 3.20.2)
project(optimiluca
  VERSION 0.0.1
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Dependencies
include(3rdparty/find_dependencies.cmake)

# Libraries
# add_subdirectory(src/optilib)

# Targets
add_subdirectory(src/apps)

### TEST
# enable_testing()
# add_subdirectory(test)

