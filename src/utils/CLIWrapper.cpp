#include "CLIWrapper.hpp"
#include "CLI11.hpp"

namespace optilib {

int CommandLineArguments::Initialize(int argc, char **argv) {
  CLI::App cli_app("Optimiluca: Optimizer for a better life.");
  argv = cli_app.ensure_utf8(argv);

  cli_app.add_option("-s, --state-size", state_size,
                     "Number of rotations to consider.");
  cli_app.add_option("-c, --n-closures", n_closures,
                     "# closures in the measurements.");
  cli_app.add_option("-i, --max-iters", max_iters,
                     "Max numbers of iterations for the solver.");
  cli_app.add_option("-d, --use-dogleg", use_dogleg,
                     "If you want to use dogleg solver or not.");
  cli_app.add_option("-v, --verbose", verbose, "Set verbosity true or false.");

  CLI11_PARSE(cli_app, argc, argv);

  // Check that the given arguments are correct
  check_args();

  return 0;
}

void CommandLineArguments::check_args() {
  // state_size
  if (state_size < 1) {
    std::cerr << "OPTIMIERROR: number of states should be at least 1!"
              << std::endl;
    exit(1);
  }

  // n_closures
  if (n_closures > state_size) {
    std::cerr
        << "OPTIMIERROR: Impossible to have a number of closures higher than "
           "state size!"
        << std::endl;
    exit(1);
  }

  // max_iters
  if (max_iters < 1) {
    std::cerr << "OPTIMIERROR: number of max iterations should be at least 1!"
              << std::endl;
    exit(1);
  }
}

} // namespace optilib
