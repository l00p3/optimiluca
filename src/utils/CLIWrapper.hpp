#pragma once

namespace optilib {

struct CommandLineArguments {

  int Initialize(int argc, char **argv);

  void check_args();

  int state_size = 8;
  int n_closures = 1;
  int max_iters = 100;
  bool use_dogleg = false;
  bool verbose = true;
};

} // namespace optilib
