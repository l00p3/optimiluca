#include <iostream>
#include <limits>

#include "Solver.hpp"

namespace optilib {

Solver::Solver(){};

std::tuple<std::vector<Rot2D>, std::vector<float>>
Solver::solve(const State &state, const std::vector<double> &measurements,
              const int n_iters) {

  // Initialization
  std::vector<double> chi_stats =
      std::vector<double>(std::numeric_limits<double>::infinity(), n_iters);

  auto J = Eigen::Matrix<double, 1, 4>::Zero();
  std::cout << J.transpose() * J << std::endl;

  // For each iterations:
  for (int iter = 0; iter < n_iters; ++iter) {

    // Initialization
    double current_chi = 0.0;

    // Initialize H and b
    auto H = Eigen::Matrix4d::Identity();
    auto b = Eigen::Vector4d::Zero();

    // For each measurements
    for (size_t meas_idx = 0; meas_idx < measurements.size(); meas_idx++) {

      // Compute the error and jacobian
      /* auto [error, J_i] = */
      /*     computeErrorAndJacobian(state[(meas_idx + 1) % 4], state[meas_idx],
       */
      /*                             Rot2D(measurements[meas_idx])); */

      // Update the stats
      /* current_chi += error * error; */

      // Update H and b
      /* H.block<4, 1>(0, (meas_idx + 1) % 4) = J_i; */
    }

    // Update the stats
    chi_stats[iter] = current_chi;

    // Update the state (boxplus)
  }

  // Done

  return {{}, {}};
};

/* std::tuple<double, Eigen::Matrix4d> */
/* Solver::computeErrorAndJacobian(const Rot2D &x_i, const Rot2D &x_j, */
/*                                 const Rot2D &z_i) const { */

/*   // Initialization */
/*   auto J_i = Eigen::Vector4d::Identity(); */

/*   // Compute the error */

/*   // Compute the Jacobian */
/* } */

} // namespace optilib
