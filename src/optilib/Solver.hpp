#pragma once

#include "State.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

namespace optilib {

class Solver {
public:
  Solver(const double &trust_region_radius = 1e4, const double epsilon = 1e-10)
      : _trust_region_radius(trust_region_radius), _epsilon(epsilon){};

  State solveWithGaussNewton(const State &state,
                             const std::vector<Measurement> &measurements,
                             const int n_iters = 100,
                             const bool verbose = false);
  State solveWithDogLeg(const State &state,
                        const std::vector<Measurement> &measurements,
                        const int n_iters = 100, const bool verbose = false);

private:
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
  bool _checkTermination(const double &state_norm);

  double _trust_region_radius;
  double _epsilon; // For termination
  Eigen::VectorXd _h_dl;
  Eigen::VectorXd _h_gn;
  Eigen::VectorXd _alpha_h_sd;
  double _alpha;
  double _h_dl_norm;
  double _linear_decrease;
  double _update_ratio;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> _sparse_solver;
};

class GNSolver {
public:
  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const int n_iters = 100,
                            const bool verbose = false);
};

class DLSolver {
public:
  DLSolver(const double &trust_region_radius = 1e4,
           const double epsilon = 1e-10)
      : _trust_region_radius(trust_region_radius), _epsilon(epsilon){};

  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const int n_iters = 10, const bool verbose = false);

private:
  double _trust_region_radius;
  double _epsilon;
  Eigen::VectorXd _h_dl;
  double _h_dl_norm;
};

} // namespace optilib
