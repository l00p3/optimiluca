#pragma once

#include <eigen3/Eigen/Dense>
#include <ostream>

using namespace std::numbers;

namespace optilib {

struct Measurement {
  Eigen::Matrix4d z;
  int from;
  int to;
};

class State {
public:
  // Constructors
  State() : _T_matrices({}){};
  State(std::vector<Eigen::Matrix4d> &&T_matrices);
  State(const size_t size);

  // Methods
  State boxPlus(const Eigen::VectorXd &dx) const;
  double distance(const State &other) const;

  // Operators
  inline constexpr size_t size() const { return _T_matrices.size(); }
  double norm() const;

  Eigen::Matrix4d &operator()(const int idx) { return _T_matrices.at(idx); }

  const Eigen::Matrix4d &operator()(const int idx) const {
    return _T_matrices.at(idx);
  }

  friend std::ostream &operator<<(std::ostream &os, const State &state);

  // Static methods
  static std::tuple<State, std::vector<Measurement>>
  generateStateAndMeasurements(const int state_size, const int n_closures);

private:
  std::vector<Eigen::Matrix4d> _T_matrices;
};

} // namespace optilib
