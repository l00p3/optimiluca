#include <iostream>
#include <limits>

#include "Solver.hpp"

namespace optilib {

Solver::Solver(){};

std::tuple<State, std::vector<float>>
Solver::solve(const State &state, const std::vector<double> &measurements,
              const int n_iters) {

  // Initialization
  const size_t state_size = state.size();
  std::vector<double> chi_stats =
      std::vector<double>(n_iters, std::numeric_limits<double>::infinity());

  // Use this as reference for Hessian computation (and error and jacobian)
  /* auto J = Eigen::Matrix<double, 1, 4>::Zero(); */
  /* std::cout << J.transpose() * J << std::endl; */

  // For each iterations:
  for (int iter = 0; iter < n_iters; ++iter) {

    // Initialization
    double current_chi = 0.0;

    // Initialize H and b
    auto H = Eigen::Matrix4d::Identity();
    auto b = Eigen::Vector4d::Zero();

    // For each measurements
    for (size_t meas_idx = 0; meas_idx < measurements.size(); meas_idx++) {

      // Compute the useful state indices
      const int observer_id = meas_idx;
      const int observed_id = (meas_idx + 1) % state_size;

      // Compute the error and jacobian
      auto [error, J_i] = computeErrorAndJacobian(
          state, observer_id, observed_id, Rot2D(measurements[meas_idx]));

      // Update the stats
      current_chi += error * error;

      // Update H and b
      /* H.block<4, 1>(0, (meas_idx + 1) % 4) = J_i.transpose() * J_i; */
      // TODO: check H and compute b
    }

    // Update the stats
    chi_stats[iter] = current_chi;

    // Update the state (boxplus)
    // We fix the first state to avoid an underconstrained problem
    // TODO: compute also dx
  }

  // Done
  return {state, {}};
};

std::tuple<double, RowVec4D>
Solver::computeErrorAndJacobian(const State &state, const int &observer_id,
                                const int &observed_id,
                                const Rot2D &z_i) const {

  // Check index validity
  assert(observed_id >= 0 && observed_id < state.size() &&
         "The value of observed_id is invalid");
  assert(observer_id >= 0 && observer_id < state.size() &&
         "The value of observer_id is invalid");

  // Initialization
  RowVec4D J_i = RowVec4D().Zero();

  // Compute the error
  double error = 1.0;

  // Compute the Jacobian // TODO: check values
  J_i(0, observer_id) = -1.0;
  J_i(0, observed_id) = 1.0;

  return {error, J_i};
}

} // namespace optilib
