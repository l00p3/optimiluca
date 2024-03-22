#pragma once

#include "State.hpp"

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

// Some rename of Eigen matrices
namespace Eigen {
using Vector6d = Vector<double, 6>;
using Vector12d = Vector<double, 12>;
using Vector36d = Vector<double, 36>;
using Matrix12_6d = Matrix<double, 12, 6>;
} // namespace Eigen

namespace optilib {

class Solver {
public:
  // --- CONSTRUCTORS --
  Solver(const double &trust_region_radius = 1e4, const double epsilon = 1e-10)
      : _trust_region_radius(trust_region_radius), _epsilon(epsilon){};

  // -- MAIN PUBLIC METHODS ---
  State solveWithGaussNewton(const State &state,
                             const std::vector<Measurement> &measurements,
                             const int n_iters = 100,
                             const bool verbose = false);
  State solveWithDogLeg(const State &state,
                        const std::vector<Measurement> &measurements,
                        const int n_iters = 100, const bool verbose = false);

private:
  // --- MAIN PRIVATE METHODS ---
  double _computeDogLegStep(const State &state,
                            const std::vector<Measurement> &measurements,
                            const int iter);
  void _computeGaussNewtonSolution(const Eigen::SparseMatrix<double> &H,
                                   const Eigen::VectorXd &b,
                                   const bool compute_sparse_solver);
  void _computeCauchyPoint(const Eigen::SparseMatrix<double> &H,
                           const Eigen::VectorXd &b,
                           const double &b_squared_norm);
  double _computeBeta(const double &alpha_h_sd_norm);
  void _updateState(State &state, const std::vector<Measurement> &measurements,
                    const double &current_chi);
  void _updateTrustRegionRadius();
  bool _checkTerminationGaussNewton(const double &h_gn_norm);
  bool _checkTerminationDogLeg(const double &state_norm);

  // --- VERBOSE METHODS ---
  void _printStart();
  void _printIterationInfoGaussNewton(const int iter,
                                      const double &current_chi);
  void _printIterationInfoDogLeg(const int iter, const double &current_chi);
  void _printTerminationInfo(const double &chi_square,
                             const double &avg_execution_time);

  // --- EXECUTION TIME METHODS ---
  void _startTimer(const int &n_iters);
  void _updateTimerIterationStarted();
  void _updateTimerIterationFinished();
  double _stopTimer();

  // --- MAIN ATTRIBUTES ---
  double _trust_region_radius;
  double _epsilon; // For termination
  Eigen::VectorXd _h_dl;
  Eigen::VectorXd _h_gn;
  Eigen::VectorXd _alpha_h_sd;
  double _alpha;
  double _h_dl_norm;
  double _linear_decrease;
  double _update_ratio;
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> _sparse_solver;

  // --- EXECUTION TIME ATTRIBUTES ---
  std::chrono::time_point<std::chrono::high_resolution_clock> _t1, _t2;
  std::vector<double> _execution_times;
};

} // namespace optilib
