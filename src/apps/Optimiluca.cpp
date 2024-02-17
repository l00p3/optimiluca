#include <iostream>
#include <numeric>
#include <vector>

#include <Solver.hpp>
#include <State.hpp>

using namespace optilib;
using namespace std::numbers;

int main() {
  // Initialization
  Solver solver;
  std::vector<double> ground_truth = {
      0.0, pi / 2, pi, pi + (pi / 2), pi + pi, pi + pi + (pi / 2)};

  // Define the initial guess
  /* auto state = State({0.0, 0.0, 0.0, 0.0}); */
  auto state = State({0.0, 0.0, 0.0, 1.6, 0.0, 0.0});
  /* auto state = State({0.0, pi / 2 + 1.4, pi - 1.2, pi + 1.3}); */
  /* auto state = State(ground_truth); */

  std::vector<double> measurements = {pi / 2, pi / 2, pi / 2,
                                      pi / 2, pi / 2, -pi / 2};

  // Optimize
  auto chi_stats = solver.solve(state, measurements, 10, true);

  // VERBOSE
  std::cout << std::endl << "Ground truth: " << std::endl;
  std::cout << State(ground_truth) << std::endl;

  std::cout << std::endl << "Optimized state: " << std::endl;
  std::cout << state << std::endl;
}
