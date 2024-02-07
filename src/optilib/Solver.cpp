#include <Solver.hpp>

namespace optilib {

Solver::Solver(){};

std::tuple<std::vector<Rot2D>, std::vector<float>>
Solver::solve(const std::vector<Rot2D> &state,
              const std::vector<Rot2D> &measurements, const int n_iters) {

  // Initialization

  // For each iterations:
  for (int iter = 0; iter < n_iters; ++iter) {

    // Initialize H and b

    // For each measurements
    for (const auto &z : measurements) {

      // Compute the error and jacobian

      // Update the stats

      // Update H and b
    }

    // Update the stats

    // Update the state (boxplus)
  }

  // Done

  return {{}, {}};
};

} // namespace optilib
