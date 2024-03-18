#include <iostream>

#include <CLIWrapper.hpp>
#include <Solver.hpp>
#include <State.hpp>

using namespace optilib;
using namespace std::numbers;

int main(int argc, char **argv) {
  // Parse the command line
  CommandLineArguments cli_args;
  cli_args.Initialize(argc, argv);

  // Initialization
  GNSolver solver;

  // Generate the ground truth and the measurements
  auto [ground_truth, measurements] = State::generateStateAndMeasurements(
      cli_args.state_size, cli_args.n_closures);

  // Define the initial guess at 0
  State state(cli_args.state_size);

  // Optimize
  auto chi_stats =
      solver.solve(state, measurements, cli_args.max_iters, cli_args.verbose);

  if (cli_args.verbose) {
    std::cout << std::endl << "Ground truth: " << std::endl;
    std::cout << State(ground_truth) << std::endl;

    std::cout << std::endl << "Optimized state: " << std::endl;
    std::cout << state << std::endl;
  }

  std::cout << std::endl
            << "Final angles error: " << state.distance(ground_truth)
            << std::endl;
}
