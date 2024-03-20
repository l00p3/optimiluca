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
  void _updateTrustRegionRadius();
  Eigen::VectorXd
  _computeGaussNewtonSolution(const Eigen::SparseMatrix<double> &H,
                              const Eigen::VectorXd &b,
                              const bool compute_sparse_solver);
  std::pair<double, Eigen::VectorXd>
  _computeCauchyPoint(const Eigen::SparseMatrix<double> &H,
                      const Eigen::VectorXd &b, const double &b_squared_norm);
  void _computeHybridPoint();
  bool _checkTermination();
  void _updateState();

  double _trust_region_radius;
  double _epsilon; // For termination
  Eigen::VectorXd _h_dl;
  double _h_dl_norm;
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
