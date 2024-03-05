#include <iostream>
#include <limits>
#include <numeric>

#include "Lumath.hpp"
#include "Solver.hpp"

// Linear System Entry Struct
namespace {
struct LinearSystemEntry {

  LinearSystemEntry(const size_t size)
      : H{Eigen::MatrixXd::Zero(size, size)}, b{Eigen::VectorXd::Zero(size)} {}

  LinearSystemEntry &operator+=(const LinearSystemEntry &rhs) {
    H += rhs.H;
    b += rhs.b;
    current_chi += rhs.current_chi;
    return *this;
  }

  friend LinearSystemEntry operator+(LinearSystemEntry lhs,
                                     const LinearSystemEntry &rhs) {
    lhs += rhs;
    return lhs;
  }

  Eigen::MatrixXd H;
  Eigen::VectorXd b;
  double current_chi = 0.0;
};
} // namespace

namespace optilib {

std::vector<double> Solver::solve(State &state,
                                  const std::vector<double> &measurements,
                                  const double termination_th,
                                  const int n_iters, const bool verbose) {

  // Initialization
  const size_t state_size = state.size();
  const size_t meas_size = measurements.size();
  std::vector<double> chi_stats(n_iters, std::numeric_limits<double>::max());
  std::vector<int> meas_indices(measurements.size());
  std::ranges::iota(meas_indices, 0);

  // VERBOSE
  if (verbose) {
    std::cout << "\n\t OPTIMIZATION STARTED ( initial guess: " << state
              << "):" << std::endl
              << std::endl;
  }

  // Function to apply to each entry of the Hessian H
  auto to_linear_system_entry = [&](const int meas_idx) {
    // Solve data association
    const int from_idx = meas_idx;
    const int to_idx = (meas_idx + 1) % state_size;

    // Compute the error and jacobian
    auto [e, J] = computeErrorAndJacobian(
        state, from_idx, to_idx, Eigen::Rotation2Dd(measurements[meas_idx]));

    // Create the entry of the linear system
    LinearSystemEntry entry(state_size);
    entry.H = J.transpose() * J;
    entry.b = J.transpose() * e;
    entry.current_chi = e.squaredNorm();

    return entry;
  };

  // For each iterations:
  for (int iter = 0; iter < n_iters; ++iter) {

    // Compute the entry of the linear system for each measurement
    const auto &[H, b, chi_square] = std::transform_reduce(
        meas_indices.cbegin(), meas_indices.cend(),
        LinearSystemEntry(state_size), std::plus<>(), to_linear_system_entry);

    // Update the stats
    chi_stats[iter] = chi_square;

    // Compute the update
    // We fix the first state to avoid an underconstrained problem
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(state_size);
    dx.tail(state_size - 1) = H.block(1, 1, state_size - 1, state_size - 1)
                                  .ldlt()
                                  .solve(-b.tail(state_size - 1));

    // Update the state
    state.boxPlus(dx);

    // VERBOSE
    if (verbose) {
      std::cout << "\t ITER: " << iter + 0 << ", CHI: " << chi_square
                << ", state: " << state << std::endl;
    }

    // Termination
    if (chi_square < termination_th) {
      break;
    }
  }

  if (verbose) {
    std::cout << std::endl
              << "\t TERMINATED WITH ERROR: " << chi_stats.back() << std::endl;
  }

  // Done
  return chi_stats;
};

// --- UTILITY FUNCTIONS ---
std::tuple<Eigen::Vector4d, Eigen::MatrixXd>
Solver::computeErrorAndJacobian(const State &state, const int from_idx,
                                const int to_idx,
                                const Eigen::Rotation2Dd &z_i) const {

  // Compute the error
  Eigen::Vector4d error =
      flatten(state(from_idx).inverse() * state(to_idx)) - flatten(z_i);

  // Compute the Jacobian
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(4, state.size());
  J.col(from_idx) =
      flatten(rotationDerivative(state(from_idx)).transpose() * state(to_idx));
  J.col(to_idx) =
      flatten(state(from_idx).inverse() * rotationDerivative(state(to_idx)));

  return {error, J};
}

} // namespace optilib
