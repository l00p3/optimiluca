#include <chrono>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <iostream>
#include <numeric>
#include <ranges>

#include "Lumath.hpp"
#include "Solver.hpp"
#include "State.hpp"

// Linear System Entry Struct
namespace {
using namespace optilib;

using LinearSystem =
    std::tuple<Eigen::SparseMatrix<double>, Eigen::VectorXd, double>;

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

LinearSystem buildLinearSystem(const optilib::State &state,
                               const std::vector<Measurement> &measurements) {
  // Initialization
  const size_t state_size = state.size();
  std::vector<Eigen::Triplet<double>> H_triplets;
  Eigen::SparseMatrix<double> H(state_size, state_size);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(state_size);
  double chi_square = 0.0;

  // Fill the linear system
  H_triplets.reserve(measurements.size() * 4);
  std::for_each(
      measurements.cbegin(), measurements.cend(), [&](const Measurement &meas) {
        // Compute the error and jacobian
        auto [e, J_from, J_to] = computeErrorAndJacobian(state, meas);

        // Fill Heassian
        H_triplets.emplace_back(meas.from, meas.from,
                                J_from.transpose() * J_from);
        H_triplets.emplace_back(meas.from, meas.to, J_from.transpose() * J_to);
        H_triplets.emplace_back(meas.to, meas.from, J_to.transpose() * J_from);
        H_triplets.emplace_back(meas.to, meas.to, J_to.transpose() * J_to);

        // Fill b
        b(meas.from) += J_from.transpose() * e;
        b(meas.to) += J_to.transpose() * e;

        // Compute error
        chi_square += e.squaredNorm();
      });

  H.setFromTriplets(H_triplets.begin(), H_triplets.end());

  return {H, b, chi_square};
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
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> sparse_solver;
  Eigen::VectorXd dx = Eigen::VectorXd::Zero(state_size);

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
    if (iter == 0) {
      sparse_solver.compute(H.block(1, 1, state_size - 1, state_size - 1));
    }
    dx.tail(state_size - 1) = sparse_solver.solve(-b.tail(state_size - 1));

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
