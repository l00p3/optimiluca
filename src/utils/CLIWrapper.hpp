#pragma once

struct CommandLineArguments {

  int Initialize(int argc, char **argv);

  void check_args();

  int state_size = 8;
  int n_closures = 1;
  double termination_th = 1e-5;
  int max_iters = 100;
  int verbose_level = 1;
};
