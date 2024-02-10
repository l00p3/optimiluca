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

  // Define the state
  auto state = State(0.0, 0.0, 0.0, 0.0);

  // Define the measurements
  auto measurements = {M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2};

  // Optimize
  auto [new_state, chi_stats] = solver.solve(state, measurements);

  std::cout << std::endl << "Optimized state: " << std::endl;
  std::cout << new_state << std::endl;

  // TODO: plot the chi stats
}
