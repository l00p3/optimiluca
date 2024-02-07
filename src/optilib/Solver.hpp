#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Rotation2D.h>

namespace optilib {

using Rot2D = Eigen::Rotation2D<float>;

class Solver {
public:
  // Constructors
  Solver();
  void init();

  std::tuple<std::vector<Rot2D>, std::vector<float>>
  solve(const std::vector<Rot2D> &state, const std::vector<Rot2D> &measurements,
        const int n_iters = 10);

private:
  // Utility functions
  void computeErrorAndJacobian(const Rot2D &x_i, const Rot2D &x_j,
                               const Rot2D &z_i) const;

  void boxPlus() const;

  // Here you put all the parameters
};

} // namespace optilib
