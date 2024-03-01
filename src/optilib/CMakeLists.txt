add_library(optilib)
target_sources(optilib PRIVATE 
  State.cpp
  Lumath.cpp
  Solver.cpp)
target_compile_features(optilib PUBLIC cxx_std_23)
target_include_directories(optilib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(optilib PRIVATE Eigen3::Eigen)
