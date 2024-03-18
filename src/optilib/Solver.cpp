#include <chrono>
#include <cmath>
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
  H_triplets.reserve((measurements.size() * 4) + 1);
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

  // Fix the first state by assigning a very high certainty and b(0) = 0
  H_triplets.emplace_back(0, 0, 1e20);
  b(0) = 0.0;

  // Build the sparse system
  H.setFromTriplets(H_triplets.begin(), H_triplets.end());

  return {H, b, chi_square};
}
} // namespace

namespace optilib {

std::vector<double>
GSSolver::solve(State &state, const std::vector<Measurement> &measurements,
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
      sparse_solver.compute(H);
    }
    dx = sparse_solver.solve(-b);

    // Update the stats
    t2 = std::chrono::high_resolution_clock::now();
    execution_times.emplace_back((t2 - t1).count());
    chi_stats.emplace_back(current_chi);

    // Update the state
    state = state.boxPlus(dx);

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

std::vector<double>
DLSolver::solve(State &state, const std::vector<Measurement> &measurements,
                const int n_iters, const int verbose_level) {
  // Initialization
  const size_t state_size = state.size();
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> sparse_solver;
  Eigen::VectorXd dx_gn = Eigen::VectorXd::Zero(state_size);
  Eigen::VectorXd dx_sd = Eigen::VectorXd::Zero(state_size);
  double linear_decrease;

  // For each iteration
  for (int iter = 0; iter < n_iters; ++iter) {

    // Compute Hessian and gradient
    const auto [H, b, current_chi] = buildLinearSystem(state, measurements);
    const double b_norm = b.norm();

    // Compute Gauss-Newton Direction
    if (iter == 0) {
      sparse_solver.compute(H);
    }
    dx_gn = sparse_solver.solve(-b);

    // Comput the Cauchy point
    const double alpha = (b.norm() / (b.transpose() * H * b));
    dx_sd = alpha * b;
    double dx_sd_norm = dx_sd.norm();

    // Check if it is inside the trust region, if yes accept it
    if (dx_gn.norm() <= this->_trust_region_radius) {
      // Take dx_gn as solution
      this->_dx = dx_gn;
      this->_dx_norm = this->_dx.norm();
      std::cout << "GN dx" << std::endl;

      // Compute the linear decrease
      linear_decrease = 0.5 * current_chi;

      // Check if it is outside the trust region, if yes take the rescaled
    } else if (dx_sd_norm >= this->_trust_region_radius) {
      // Take the intersection of the "leg" to dx_sd with the trust region
      this->_dx = this->_trust_region_radius * dx_sd / dx_sd_norm;
      this->_dx_norm = this->_dx.norm();
      std::cout << "SD dx" << std::endl;

      // Compute the linear decrease
      linear_decrease = this->_trust_region_radius *
                        (2 * dx_sd_norm - this->_trust_region_radius) /
                        (2 * alpha);

    } else {
      // Precomputation
      const double c = dx_sd.transpose() * (dx_gn - dx_sd);
      const double radius_minus_dx_sd =
          this->_trust_region_radius * this->_trust_region_radius -
          dx_sd_norm * dx_sd_norm;
      const double norm_difference = (dx_gn - dx_sd).norm();
      const double squared_norm_difference = norm_difference * norm_difference;
      const double squared_term =
          std::sqrt(c * c + squared_norm_difference * radius_minus_dx_sd);

      const double beta = (c <= 0)
                              ? radius_minus_dx_sd / (c + squared_term)
                              : (-c + squared_term) / squared_norm_difference;

      // Take the intersection of the "leg" between dx_gn and dx_sd
      this->_dx = dx_sd + beta * (dx_gn - dx_sd);
      this->_dx_norm = this->_dx.norm();
      std::cout << "GN/SD dx" << std::endl;

      // Compute the linear decrease
      linear_decrease =
          0.5 * alpha * (1 - beta) * (1 - beta) * b_norm * b_norm +
          beta * (2 - beta) * current_chi;
    }

    // Compute the update

    // TODO:
    // 1. You should find a way to have an updated version of the current state,
    // that you can discard if the update is not good enough
    // 2. You should comute H, b and chi_square and reuse it in the next
    // itaration

    // Check goodness of the step
    /* if (this->_dx_norm <= th) { */
    // TODO: apply perturbation
    /* } else { */
    // Do all the rest
    /* } */

    // Apply the update
    state.boxPlus(this->_dx);

    // Compute the ratio for update
    /* const double ratio = */
    /*     (0.5 * current_chi - 0.5 * updated_chi) / linear_decrease; */

    // Rescale the trust region radius
    // TODO
  }

  return {0.0};
}

} // namespace optilib
