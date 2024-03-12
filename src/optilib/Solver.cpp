#include <chrono>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <execution>
#include <iostream>
#include <numeric>

#include "Lumath.hpp"
#include "Solver.hpp"
#include "State.hpp"

// Linear System Entry Struct
namespace {
using namespace optilib;

struct LinearSystemEntry {

  LinearSystemEntry(const size_t size)
      : H{Eigen::SparseMatrix<double>(size, size)},
        b{Eigen::VectorXd::Zero(size)} {}

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

  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd b;
  double current_chi = 0.0;
};

std::tuple<Eigen::Vector4d, Eigen::VectorXd, Eigen::VectorXd>
computeErrorAndJacobian(const State &state, const Measurement &meas) {

  // Compute the error
  Eigen::Vector4d error =
      flatten(state(meas.from).inverse() * state(meas.to)) - flatten(meas.z);

  // Compute the Jacobian
  Eigen::VectorXd J_from = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd J_to = Eigen::VectorXd::Zero(4);
  J_from = flatten(rotationDerivative(state(meas.from)).transpose() *
                   state(meas.to));
  J_to =
      flatten(state(meas.from).inverse() * rotationDerivative(state(meas.to)));

  return {error, J_from, J_to};
}

LinearSystemEntry
buildLinearSystem(const optilib::State &state,
                  const std::vector<Measurement> &measurements) {
  // Initialization
  const size_t state_size = state.size();

  // Fill the linear system
  return std::transform_reduce(
      std::execution::par, measurements.cbegin(), measurements.cend(),
      LinearSystemEntry(state_size), std::plus<>(),
      [&](const Measurement &meas) {
        // Compute the error and jacobian
        auto [e, J_from, J_to] = computeErrorAndJacobian(state, meas);

        // Initialize the linear system entry
        LinearSystemEntry linear_system_entry(state_size);

        // Fill the non-zero entries
        std::vector<Eigen::Triplet<double>> H_triplets(4);

        H_triplets[0] = {meas.from, meas.from, J_from.transpose() * J_from};
        H_triplets[1] = {meas.from, meas.to, J_from.transpose() * J_to};
        H_triplets[2] = {meas.to, meas.from, J_to.transpose() * J_from};
        H_triplets[3] = {meas.to, meas.to, J_to.transpose() * J_to};
        linear_system_entry.H.setFromTriplets(H_triplets.begin(),
                                              H_triplets.end());

        linear_system_entry.b(meas.from) = J_from.transpose() * e;
        linear_system_entry.b(meas.to) = J_to.transpose() * e;
        linear_system_entry.current_chi = e.squaredNorm();

        return linear_system_entry;
      });
}
} // namespace

namespace optilib {

std::vector<double> Solver::solve(State &state,
                                  const std::vector<Measurement> &measurements,
                                  const int n_iters, const int verbose_level) {

  // Initialization
  const size_t state_size = state.size();
  std::vector<double> chi_stats;
  std::vector<double> execution_times;
  std::chrono::time_point<std::chrono::high_resolution_clock> t1, t2;

  // VERBOSE
  if (verbose_level) {
    std::cout << "\n\t OPTIMIZATION STARTED";
    if (verbose_level == 2) {
      std::cout << " ( initial guess: " << state << ")";
    }
    std::cout << std::endl << std::endl;
  }

  // For each iterations
  chi_stats.reserve(n_iters);
  execution_times.reserve(n_iters);
  for (int iter = 0; iter < n_iters; ++iter) {
    t1 = std::chrono::high_resolution_clock::now();

    // Compute the entry of the linear system for each measurement
    const auto [H, b, current_chi] = buildLinearSystem(state, measurements);

    // Compute the update: fixing the first state to origin
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(state_size);
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> chol_H(
        H.block(1, 1, state_size - 1, state_size - 1));
    dx.tail(state_size - 1) = chol_H.solve(-b.tail(state_size - 1));

    // Update the stats
    t2 = std::chrono::high_resolution_clock::now();
    execution_times.emplace_back((t2 - t1).count());
    chi_stats.emplace_back(current_chi);

    // Update the state
    state.boxPlus(dx);

    // VERBOSE
    if (verbose_level) {
      std::cout << "\t ITER: " << iter + 0 << ", CHI SQUARE: " << current_chi
                << std::endl;
      if (verbose_level == 2) {
        std::cout << "\t\t State: " << state << std::endl;
      }
    }

    // Termination criteria
    if (dx.norm() < 1e-10)
      break;
  }
  chi_stats.shrink_to_fit();
  execution_times.shrink_to_fit();

  // VERBOSE
  if (verbose_level) {
    const double avg_execution_time =
        (std::accumulate(execution_times.begin(), execution_times.end(), 0.0) /
         execution_times.size()) *
        1e-9;
    std::cout << std::endl
              << "\t TERMINATED WITH CHI SQUARE: " << chi_stats.back()
              << " (iter. avg. time: " << avg_execution_time << "sec)"
              << std::endl
              << std::setprecision(3);
    ;
  }

  // Done
  return chi_stats;
};

// --- UTILITY FUNCTIONS ---

} // namespace optilib
