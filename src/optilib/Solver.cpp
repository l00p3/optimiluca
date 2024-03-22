#include <cmath>
#include <iostream>
#include <numeric>
#include <ranges>
#include <utility>
#include <vector>

#include "Lumath.hpp"
#include "Solver.hpp"
#include "State.hpp"

// --- Utility functions in unnamed namespace ---
namespace {
using namespace optilib;

using LinearSystem =
    std::tuple<Eigen::SparseMatrix<double>, Eigen::VectorXd, double>;

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

std::tuple<Eigen::VectorXd, Eigen::MatrixXd>
computeErrorAndJacobian(const State &state, const Measurement &meas) {

  // Initialize some reference for readability
  const Eigen::Matrix3d R_from_transpose =
      state(meas.from).block<3, 3>(0, 0).transpose();
  const Eigen::Matrix3d &R_to = state(meas.to).block<3, 3>(0, 0);
  const Eigen::Vector3d &t_to = state(meas.to).block<3, 1>(0, 3);

  // Compute the error
  Eigen::VectorXd error =
      flatten(T_inverse(state(meas.from)) * state(meas.to) - meas.z);

  // Compute the Jacobian
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(12, 6);
  J.block<9, 1>(0, 3) = (R_from_transpose * Rx_der_0() * R_to).reshaped();
  J.block<9, 1>(0, 4) = (R_from_transpose * Ry_der_0() * R_to).reshaped();
  J.block<9, 1>(0, 5) = (R_from_transpose * Rz_der_0() * R_to).reshaped();
  J.block<3, 3>(9, 0) = R_from_transpose;
  J.block<3, 3>(9, 3) = -R_from_transpose * skew(t_to);

  return {error, J};
}

LinearSystem buildLinearSystem(const State &state,
                               const std::vector<Measurement> &measurements) {
  // Initializatio-n
  const size_t state_size = state.size();
  std::vector<Eigen::Triplet<double>> H_triplets;
  Eigen::SparseMatrix<double> H(state_size * 6, state_size * 6);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(state_size * 6);
  double chi_square = 0.0;

  // Fill the linear system
  H_triplets.reserve((measurements.size() * 36 * 4) + 1);
  std::for_each(
      measurements.cbegin(), measurements.cend(), [&](const Measurement &meas) {
        // Compute the error and jacobian
        const auto [e, J] = computeErrorAndJacobian(state, meas);
        const Eigen::VectorXd J_transpose_J =
            (J.transpose() * J).reshaped(); // Vectorized for easier loop
        const Eigen::VectorXd J_transpose_e = J.transpose() * e;

        // Fill Hessian
        std::ranges::for_each(
            std::views::enumerate(J_transpose_J), [&](const auto &idx_val) {
              const auto &[idx, val] = idx_val;
              const int J_row = idx % 6; // from vec to matrix position
              const int J_col = idx / 6;
              H_triplets.emplace_back((meas.from * 6) + J_row,
                                      (meas.from * 6) + J_col, val);
              H_triplets.emplace_back((meas.from * 6) + J_row,
                                      (meas.to * 6) + J_col, -val);
              H_triplets.emplace_back((meas.to * 6) + J_row,
                                      (meas.from * 6) + J_col, -val);
              H_triplets.emplace_back((meas.to * 6) + J_row,
                                      (meas.to * 6) + J_col, val);
            });

        // Fill b
        b.segment<6>(meas.from * 6) += J_transpose_e;
        b.segment<6>(meas.to * 6) -= J_transpose_e;

        // Compute error
        chi_square += e.squaredNorm();
      });

  // Fix the first state by assigning a very high certainty and b(0) = 0
  for (int i = 0; i < 6; i++) {
    H_triplets.emplace_back(i, i, 1e200);
    // TODO set b to 0?
  }

  // Build the sparse system
  H.setFromTriplets(H_triplets.begin(), H_triplets.end());

  return {H, -b, chi_square};
}

} // namespace

// --- METHODS IMPLEMENTATIONS ---
namespace optilib {

// --- MAIN PUBLIC METHODS ---
State Solver::solveWithGaussNewton(const State &state,
                                   const std::vector<Measurement> &measurements,
                                   const int n_iters, const bool verbose) {
  // Initialization
  State optimized_state = state;
  const double state_size = state.size();
  double current_chi = 0.0;
  this->_startTimer(n_iters);

  if (verbose)
    this->_printStart();

  // For each iteration
  for (int iter = 0; iter < n_iters; ++iter) {
    // Execution time
    this->_updateTimerIterationStarted();

    // Compute Hessian and gradient
    const auto [H, b, current_chi] =
        buildLinearSystem(optimized_state, measurements);

    // Compute Gauss-Newton Direction
    _computeGaussNewtonSolution(H, b, iter == 0);

    // Update the state
    optimized_state = optimized_state.boxPlus(this->_h_gn);

    // Execution time
    this->_updateTimerIterationFinished();

    // VERBOSE
    if (verbose) {
      this->_printIterationInfoGaussNewton(iter, current_chi);
    }

    // Termination criteria
    if (this->_checkTerminationGaussNewton(this->_h_gn.norm()))
      break;
  }

  // Execution time
  const double avg_execution_time = this->_stopTimer();

  // VERBOSE
  if (verbose) {
    this->_printTerminationInfo(current_chi, avg_execution_time);
  }

  return std::move(optimized_state);
}

State Solver::solveWithDogLeg(const State &state,
                              const std::vector<Measurement> &measurements,
                              const int n_iters, const bool verbose) {

  // Initialization
  State optimized_state = state;
  const double state_size = state.size();
  double current_chi = 0.0;
  this->_startTimer(n_iters);

  // Initialize direction vectors sizes
  this->_h_gn = Eigen::VectorXd::Zero(state_size);
  this->_alpha_h_sd = Eigen::VectorXd::Zero(state_size);

  if (verbose)
    this->_printStart();

  // For each iteration
  for (int iter = 0; iter < n_iters; ++iter) {
    // Execution time
    this->_updateTimerIterationStarted();

    current_chi = this->_computeDogLegStep(optimized_state, measurements, iter);

    this->_updateState(optimized_state, measurements, current_chi);

    this->_updateTrustRegionRadius();

    // Execution time
    this->_updateTimerIterationFinished();

    // VERBOSE
    if (verbose) {
      this->_printIterationInfoDogLeg(iter, current_chi);
    }

    if (this->_checkTerminationDogLeg(optimized_state.norm()))
      break;
  }

  // Execution time
  const double avg_execution_time = this->_stopTimer();

  // VERBOSE
  if (verbose) {
    this->_printTerminationInfo(current_chi, avg_execution_time);
  }

  return std::move(optimized_state);
}

// --- MAIN PRIVATE METHODS ---
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
    this->_h_dl = this->_h_gn;
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

bool Solver::_checkTerminationGaussNewton(const double &h_gn_norm) {
  return h_gn_norm < 1e-10;
}

bool Solver::_checkTerminationDogLeg(const double &state_norm) {
  return this->_h_dl_norm <= this->_epsilon * (state_norm + this->_epsilon) ||
         this->_trust_region_radius <=
             this->_epsilon * (state_norm + this->_epsilon);
}

// --- VERBOSE METHODS ---
void Solver::_printStart() {
  std::cout << "\n\t OPTIMIZATION STARTED";
  std::cout << std::endl << std::endl;
}

void Solver::_printIterationInfoGaussNewton(const int iter,
                                            const double &current_chi) {
  std::cout << "\t ITER: " << iter + 0 << ", CHI SQUARE: " << current_chi
            << std::endl;
}

void Solver::_printIterationInfoDogLeg(const int iter,
                                       const double &current_chi) {
  std::cout << "\t ITER: " << iter + 0 << ", \tCHI SQUARE: " << current_chi
            << ", \tradius: " << this->_trust_region_radius
            << ", \tratio: " << this->_update_ratio << std::endl;
}

void Solver::_printTerminationInfo(const double &chi_square,
                                   const double &avg_execution_time) {
  std::cout << std::endl
            << "\t TERMINATED WITH CHI SQUARE: " << chi_square
            << " (iter. avg. time: " << avg_execution_time << "sec)"
            << std::endl
            << std::setprecision(3);
}

// --- EXECUTION TIME ATTRIBUTES ---
void Solver::_startTimer(const int &n_iters) {
  this->_execution_times = {};             // Empty the vector
  this->_execution_times.reserve(n_iters); // Reserve size
}

void Solver::_updateTimerIterationStarted() {
  this->_t1 = std::chrono::high_resolution_clock::now();
}

void Solver::_updateTimerIterationFinished() {
  this->_t2 = std::chrono::high_resolution_clock::now();
  this->_execution_times.emplace_back((this->_t2 - this->_t1).count());
}

double Solver::_stopTimer() {
  this->_execution_times.shrink_to_fit();
  return (std::accumulate(this->_execution_times.begin(),
                          this->_execution_times.end(), 0.0) /
          this->_execution_times.size()) *
         1e-9;
}

} // namespace optilib
