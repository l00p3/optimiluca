#include <iostream>
#include <numeric>
#include <vector>

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
  Solver solver;

  // Generate the ground truth and the measurements
  auto [ground_truth, measurements] =
      State::generateStateAndMeasurements(cli_args.state_size);

  // Define the initial guess
  std::vector<double> angles(ground_truth.size(), 0.0);
  angles[3] = ground_truth(3).angle();
  State state(angles);

  // Optimize
  auto chi_stats =
      solver.solve(state, measurements, cli_args.max_iters, cli_args.verbose);

  // VERBOSE
  std::cout << std::endl << "Ground truth: " << std::endl;
  std::cout << State(ground_truth) << std::endl;

  std::cout << std::endl << "Optimized state: " << std::endl;
  std::cout << state << std::endl;
}
