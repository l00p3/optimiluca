add_library(optilib)
target_sources(optilib PRIVATE 
  Solver.cpp)
target_include_directories(optilib PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(optilib PRIVATE Eigen3::Eigen)
