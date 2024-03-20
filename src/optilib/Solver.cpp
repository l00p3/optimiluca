#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <ranges>
#include <utility>
#include <vector>

#include "Lumath.hpp"
#include "Solver.hpp"
#include "State.hpp"

namespace {
using namespace optilib;

using Timer = std::chrono::time_point<std::chrono::high_resolution_clock>;

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
  Eigen::VectorXd h_gn = Eigen::VectorXd::Zero(state_size);
  Eigen::VectorXd alpha_h_sd = Eigen::VectorXd::Zero(state_size);
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
    h_gn = sparse_solver.solve(-b);
    const double h_gn_norm = h_gn.norm();

    // Compute the Cauchy point
    const double alpha = (b_squared_norm / (b.transpose() * H * b));
    alpha_h_sd = -alpha * b;
    const double alpha_h_sd_norm = alpha_h_sd.norm();

    // Check if it is inside the trust region, if yes accept it
    if (h_gn_norm <= this->_trust_region_radius) {
      // Take dx_gn as solution
      this->_h_dl = h_gn;
      this->_h_dl_norm = h_gn_norm;
      method_used = "Gauss-Newton";

      // Compute the linear decrease
      linear_decrease = 0.5 * current_chi;

      // Check if it is outside the trust region, if yes take the rescaled
    } else if (alpha_h_sd_norm >= this->_trust_region_radius) {
      // Take the intersection of the "leg" to dx_sd with the trust region
      this->_h_dl = this->_trust_region_radius * -b / b_norm;
      this->_h_dl_norm = this->_h_dl.norm();
      method_used = "Steepest Descent";

      // Compute the linear decrease
      linear_decrease = this->_trust_region_radius *
                        (2 * alpha_h_sd_norm - this->_trust_region_radius) /
                        (2 * alpha);

    } else {
      // Precomputation
      const double c = alpha_h_sd.transpose() * (h_gn - alpha_h_sd);
      const double radius_minus_dx_sd =
          this->_trust_region_radius * this->_trust_region_radius -
          alpha_h_sd_norm * alpha_h_sd_norm;
      const double squared_norm_difference = (h_gn - alpha_h_sd).squaredNorm();
      const double squared_term =
          std::sqrt(c * c + squared_norm_difference * radius_minus_dx_sd);

      const double beta = (c <= 0)
                              ? radius_minus_dx_sd / (c + squared_term)
                              : (-c + squared_term) / squared_norm_difference;

      // Take the intersection of the "leg" between dx_gn and dx_sd
      this->_h_dl = alpha_h_sd + beta * (h_gn - alpha_h_sd);
      this->_h_dl_norm = this->_h_dl.norm();
      method_used = "Hybrid";

      // Compute the linear decrease
      linear_decrease = 0.5 * alpha * (1 - beta) * (1 - beta) * b_squared_norm +
                        beta * (2 - beta) * (0.5 * current_chi);
    }

    // Compute the update
    State new_state = state.boxPlus(_h_dl);

    // Compute the ratio for update
    const double new_state_chi = computeChiSquare(new_state, measurements);
    const double update_ratio =
        (0.5 * current_chi - 0.5 * new_state_chi) / linear_decrease;

    // Update the trust region radius
    if (update_ratio > 0.75)
      _trust_region_radius = std::max(_trust_region_radius, 3 * _h_dl_norm);
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
    const double state_norm = state.norm();
    if (_h_dl_norm <= _epsilon * (state_norm + _epsilon) ||
        _trust_region_radius <= _epsilon * (state_norm + _epsilon)) {
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

State Solver::solveWithDogLeg(const State &state,
                              const std::vector<Measurement> &measurements,
                              const int n_iters, const bool verbose) {

  // Initialization
  State optimized_state = state;
  const double state_size = state.size();
  std::vector<double> execution_times;
  Timer t1, t2;

  // Initialize direction vectors sizes
  this->_h_gn = Eigen::VectorXd::Zero(state_size);
  this->_alpha_h_sd = Eigen::VectorXd::Zero(state_size);

  // For each iteration
  execution_times.reserve(n_iters);
  for (int iter = 0; iter < n_iters; ++iter) {
    // Execution time
    t1 = std::chrono::high_resolution_clock::now();

    const double current_chi =
        this->_computeDogLegStep(state, measurements, iter);

    this->_updateState(optimized_state, measurements, current_chi);

    this->_updateTrustRegionRadius();

    // Update the stats
    t2 = std::chrono::high_resolution_clock::now();
    execution_times.emplace_back((t2 - t1).count());

    if (this->_checkTermination(state.norm()))
      break;
  }
  execution_times.shrink_to_fit();

  return optimized_state;
}

double Solver::_computeDogLegStep(const State &state,
                                  const std::vector<Measurement> &measurements,
                                  const int iter) {
  // Compute Hessian and gradient
  const auto [H, b, current_chi] = buildLinearSystem(state, measurements);
  const double b_squared_norm = b.squaredNorm();
  const double b_norm = b.norm();

  // Compute Gauss-Newton Direction
  _computeGaussNewtonSolution(H, b, iter == 0);
  const double h_gn_norm = this->_h_gn.norm();

  // Compute the Cauchy point
  _computeCauchyPoint(H, b, b_squared_norm);
  const double alpha_h_sd_norm = this->_alpha_h_sd.norm();

  if (h_gn_norm <= this->_trust_region_radius) {
    // Case 1: Gauss-Newton
    this->_h_dl = _h_gn;
    this->_linear_decrease = 0.5 * current_chi;
  } else if (alpha_h_sd_norm >= this->_trust_region_radius) {
    // Case 2: Chauchy Point
    this->_h_dl = this->_trust_region_radius * -b / b_norm;
    this->_linear_decrease =
        this->_trust_region_radius *
        (2 * alpha_h_sd_norm - this->_trust_region_radius) / (2 * this->_alpha);
  } else {
    // Case 3: "Hybrid"
    const double beta = _computeBeta(alpha_h_sd_norm);
    this->_h_dl = _alpha_h_sd + beta * (_h_gn - _alpha_h_sd);
    this->_linear_decrease =
        0.5 * this->_alpha * (1 - beta) * (1 - beta) * b_squared_norm +
        beta * (2 - beta) * (0.5 * current_chi);
  }

  this->_h_dl_norm = this->_h_dl.norm();

  return current_chi;
}

void Solver::_computeGaussNewtonSolution(const Eigen::SparseMatrix<double> &H,
                                         const Eigen::VectorXd &b,
                                         const bool compute_sparse_solver) {
  if (compute_sparse_solver)
    _sparse_solver.compute(H);
  this->_h_gn = _sparse_solver.solve(-b);
}

void Solver::_computeCauchyPoint(const Eigen::SparseMatrix<double> &H,
                                 const Eigen::VectorXd &b,
                                 const double &b_squared_norm) {

  this->_alpha = (b_squared_norm / (b.transpose() * H * b));
  this->_alpha_h_sd = -this->_alpha * b;
}

double Solver::_computeBeta(const double &alpha_h_sd_norm) {
  const double c =
      this->_alpha_h_sd.transpose() * (this->_h_gn - this->_alpha_h_sd);
  const double radius_minus_dx_sd =
      this->_trust_region_radius * this->_trust_region_radius -
      alpha_h_sd_norm * alpha_h_sd_norm;
  const double squared_norm_difference =
      (this->_h_gn - this->_alpha_h_sd).squaredNorm();
  const double squared_term =
      std::sqrt(c * c + squared_norm_difference * radius_minus_dx_sd);

  return (c <= 0) ? radius_minus_dx_sd / (c + squared_term)
                  : (-c + squared_term) / squared_norm_difference;
}

void Solver::_updateState(State &state,
                          const std::vector<Measurement> &measurements,
                          const double &current_chi) {
  // Compute the update
  State new_state = state.boxPlus(this->_h_dl);

  // Compute the ratio for update
  const double new_state_chi = computeChiSquare(new_state, measurements);
  this->_update_ratio =
      (0.5 * current_chi - 0.5 * new_state_chi) / this->_linear_decrease;

  // Accept or refuse the update
  if (this->_update_ratio > 0) {
    // Apply the update
    state = std::move(new_state);
  }
}

void Solver::_updateTrustRegionRadius() {
  if (this->_update_ratio > 0.75)
    this->_trust_region_radius =
        std::max(this->_trust_region_radius, 3 * this->_h_dl_norm);
  else if (this->_update_ratio < 0.25)
    this->_trust_region_radius *= 0.5;
}

bool Solver::_checkTermination(const double &state_norm) {
  return this->_h_dl_norm <= this->_epsilon * (state_norm + this->_epsilon) ||
         this->_trust_region_radius <=
             this->_epsilon * (state_norm + this->_epsilon);
}

} // namespace optilib
