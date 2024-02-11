#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>

#include <optilib/Solver.hpp>
#include <optilib/State.hpp>
#include <vector>

using namespace optilib;

int main() {
  // Initialization
  auto solver = Solver();
  std::vector<double> ground_truth = {0.0, M_PI / 2, M_PI, 3 * (M_PI / 2)};

  // Define the initial guess
  /* auto state = State(0.0, M_PI / 3, M_PI / 1.2, M_PI); */
  auto state = State(0.0, M_PI / 2, M_PI, 3 * (M_PI / 2));

  // Define the measurements
  std::vector<double> measurements = {M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2};

  // Optimize
  auto chi_stats = solver.solve(state, measurements, 10, true);

  std::cout << std::endl << "Optimized state: " << std::endl;
  std::cout << state << std::endl;

  // TODO: plot the chi stats
}
