#pragma once

#include "CLI11.hpp"

struct CommandLineArguments {

  int Initialize(int argc, char **argv);

  int state_size = 8;
  int n_closures = 1;
  int max_iters = 100;
  bool verbose = false;
};
