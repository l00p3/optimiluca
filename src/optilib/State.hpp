#pragma once

#include <eigen3/Eigen/Dense>
#include <ostream>

using namespace std::numbers;

namespace optilib {

struct Measurement {
  Eigen::Rotation2Dd z;
  int from;
  int to;
};

class State {
public:
  // Constructors
  State() : _rotations({}){};
  State(const std::vector<double> &angles);
  State(std::vector<Eigen::Rotation2Dd> &&rotations);
  State(const size_t size);

  // Methods
  State boxPlus(const Eigen::VectorXd &dx) const;
  double distance(const State &other) const;

  // Operators
  inline constexpr size_t size() const { return _rotations.size(); }
  double norm() const;

  Eigen::Rotation2Dd &operator()(const int idx) { return _rotations.at(idx); }

  const Eigen::Rotation2Dd &operator()(const int idx) const {
    return _rotations.at(idx);
  }

  friend std::ostream &operator<<(std::ostream &os, const State &state);

  // Static methods
  static std::tuple<State, std::vector<Measurement>>
  generateStateAndMeasurements(const int state_size, const int n_closures);

private:
  std::vector<Eigen::Rotation2Dd> _rotations;
};

} // namespace optilib
