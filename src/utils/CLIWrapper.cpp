#include "CLIWrapper.hpp"
#include "CLI11.hpp"

int CommandLineArguments::Initialize(int argc, char **argv) {
  CLI::App cli_app("Optimiluca: Optimizer for a better life.");
  argv = cli_app.ensure_utf8(argv);

  cli_app.add_option("-s, --state-size", state_size,
                     "Number of rotations to consider.");
  cli_app.add_option("-c, --n-closures", n_closures,
                     "# closures in the measurements.");
  cli_app.add_option("-i, --max-iters", max_iters,
                     "Max numbers of iterations for the solver.");
  cli_app.add_option("-v, --verbose", verbose, "Set verbosity.");

  CLI11_PARSE(cli_app, argc, argv);

  return 0;
}
