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

void printExecutionTime(const std::vector<double> &execution_times) {
  const double avg_execution_time =
      (std::accumulate(execution_times.begin(), execution_times.end(), 0.0) /
       execution_times.size()) *
      1e-9;
  std::cout << avg_execution_time << "sec)" << std::setprecision(3);
}

double computeChiSquare(const State &state,
                        const std::vector<Measurement> &measurements) {
  return std::accumulate(
      measurements.cbegin(), measurements.cend(), 0.0,
      [&](const int chi_square, const Measurement &meas) {
        return chi_square +
               (flatten(state(meas.from).inverse() * state(meas.to)) -
                flatten(meas.z))
                   .squaredNorm();
      });
}

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

LinearSystem buildLinearSystem(const State &state,
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
GNSolver::solve(State &state, const std::vector<Measurement> &measurements,
                const int n_iters, const bool verbose) {

  // Initialization
  const size_t state_size = state.size();
  std::vector<double> chi_stats;
  std::vector<double> execution_times;
  std::chrono::time_point<std::chrono::high_resolution_clock> t1, t2;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> sparse_solver;
  Eigen::VectorXd dx = Eigen::VectorXd::Zero(state_size);

  // VERBOSE
  if (verbose) {
    std::cout << "\n\t OPTIMIZATION STARTED";
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
    if (verbose) {
      std::cout << "\t ITER: " << iter + 0 << ", CHI SQUARE: " << current_chi
                << std::endl;
    }

    // Termination criteria
    if (dx.norm() < 1e-10)
      break;
  }
  chi_stats.shrink_to_fit();
  execution_times.shrink_to_fit();

  // VERBOSE
  if (verbose) {
    std::cout << std::endl
              << "\t TERMINATED WITH CHI SQUARE: " << chi_stats.back()
              << " (iter. avg. time: ";
    printExecutionTime(execution_times);
    std::cout << std::endl << std::setprecision(3);
  }

  // Done
  return chi_stats;
};

std::vector<double>
DLSolver::solve(State &state, const std::vector<Measurement> &measurements,
                const int n_iters, const bool verbose) {
  // Initialization
  const size_t state_size = state.size();
  std::vector<double> chi_stats;
  std::vector<double> execution_times;
  std::chrono::time_point<std::chrono::high_resolution_clock> t1, t2;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> sparse_solver;
  Eigen::VectorXd dx_gn = Eigen::VectorXd::Zero(state_size);
  Eigen::VectorXd dx_sd = Eigen::VectorXd::Zero(state_size);
  double linear_decrease;
  std::string method_used = "ERROR";

  // For each iteration
  chi_stats.reserve(n_iters);
  execution_times.reserve(n_iters);
  for (int iter = 0; iter < n_iters; ++iter) {
    t1 = std::chrono::high_resolution_clock::now();

    // Compute Hessian and gradient
    const auto [H, b, current_chi] = buildLinearSystem(state, measurements);
    const double b_squared_norm = b.squaredNorm();
    const double b_norm = b.norm();

    // Compute Gauss-Newton Direction
    if (iter == 0) {
      sparse_solver.compute(H);
    }
    dx_gn = sparse_solver.solve(-b);
    double dx_gn_norm = dx_gn.norm();

    // Compute the Cauchy point
    const double alpha = (b_squared_norm / (b.transpose() * H * b));
    dx_sd = alpha * b;
    double dx_sd_norm = dx_sd.norm();

    // Check if it is inside the trust region, if yes accept it
    if (dx_gn_norm <= this->_trust_region_radius) {
      // Take dx_gn as solution
      this->_dx = dx_gn;
      this->_dx_norm = dx_gn_norm;
      method_used = "Gauss-Newton";

      // Compute the linear decrease
      linear_decrease = 0.5 * current_chi;

      // Check if it is outside the trust region, if yes take the rescaled
    } else if (dx_sd_norm >= this->_trust_region_radius) {
      // Take the intersection of the "leg" to dx_sd with the trust region
      this->_dx = this->_trust_region_radius * b / b_norm;
      this->_dx_norm = this->_dx.norm();
      method_used = "Steepest Descent";

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
      const double squared_norm_difference = (dx_gn - dx_sd).squaredNorm();
      const double squared_term =
          std::sqrt(c * c + squared_norm_difference * radius_minus_dx_sd);

      const double beta = (c <= 0)
                              ? radius_minus_dx_sd / (c + squared_term)
                              : (-c + squared_term) / squared_norm_difference;

      // Take the intersection of the "leg" between dx_gn and dx_sd
      this->_dx = dx_sd + beta * (dx_gn - dx_sd);
      this->_dx_norm = this->_dx.norm();
      method_used = "Hybrid";

      // Compute the linear decrease
      linear_decrease = 0.5 * alpha * (1 - beta) * (1 - beta) * b_squared_norm +
                        beta * (2 - beta) * (0.5 * current_chi);
    }

    // Compute the update
    State new_state = state.boxPlus(_dx);

    // Compute the ratio for update
    const double new_state_chi = computeChiSquare(new_state, measurements);
    const double update_ratio =
        (0.5 * current_chi - 0.5 * new_state_chi) / linear_decrease;

    // Update the trust region radius
    if (update_ratio > 0.75)
      _trust_region_radius = std::max(_trust_region_radius, 3 * _dx_norm);
    else if (update_ratio < 0.25)
      _trust_region_radius *= 0.5;

    // Update the stats
    t2 = std::chrono::high_resolution_clock::now();
    execution_times.emplace_back((t2 - t1).count());
    chi_stats.emplace_back(current_chi);

    // Accept or refuse the update
    if (update_ratio > 0) {
      // Apply the update
      state = std::move(new_state);
    }

    // VERBOSE
    if (verbose) {
      std::cout << "\t ITER: " << iter + 0 << ", \tCHI SQUARE: " << current_chi
                << ", \tradius: " << _trust_region_radius
                << ", \tratio: " << update_ratio
                << ", \tmethod: " << method_used << std::endl;
    }

    // Termination criteria
    if (_dx_norm <= 1e-10 ||
        _trust_region_radius <= _epsilon2 * (state.norm() + _epsilon2)) {
      break;
    }
  }
  chi_stats.shrink_to_fit();
  execution_times.shrink_to_fit();

  // VERBOSE
  if (verbose) {
    std::cout << std::endl
              << "\t TERMINATED WITH CHI SQUARE: " << chi_stats.back()
              << " (iter. avg. time: ";
    printExecutionTime(execution_times);
    std::cout << std::endl << std::setprecision(3);
  }

  // Done
  return chi_stats;
}

} // namespace optilib
