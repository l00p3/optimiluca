add_library(utils)
target_sources(utils PRIVATE
  CLIWrapper.cpp)
target_compile_features(utils PUBLIC cxx_std_23)
target_include_directories(utils PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR})
