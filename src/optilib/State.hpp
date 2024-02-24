#pragma once

#include <eigen3/Eigen/Dense>
#include <ostream>

using namespace std::numbers;

namespace optilib {

class State {
public:
  // Constructors
  State(const std::vector<double> &angles);

  // Methods
  void boxPlus(const Eigen::VectorXd &dx);

  // Operators
  inline constexpr size_t size() const { return _rotations.size(); }

  const Eigen::Rotation2Dd &operator()(const int idx) const {
    return _rotations.at(idx);
  }

  friend std::ostream &operator<<(std::ostream &os, const State &state);

  // Static methods
  static std::tuple<State, std::vector<double>>
  generateGroundTruthAndMeasurements(const int state_size);

private:
  std::vector<Eigen::Rotation2Dd> _rotations;
};

} // namespace optilib
