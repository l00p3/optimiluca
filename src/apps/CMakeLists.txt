add_executable(optimiluca Optimiluca.cpp)
target_include_directories(optimiluca PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(optimiluca PRIVATE optilib)
