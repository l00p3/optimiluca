#include <iostream>
#include <numeric>
#include <random>
#include <ranges>

#include <State.hpp>

using namespace std::numbers;

namespace optilib {
State::State(const std::vector<double> &angles) {
  _rotations.reserve(angles.size());
  std::ranges::for_each(
      angles, [&](const double &theta) { _rotations.emplace_back(theta); });
}

// ---------- METHODS ----------
void State::boxPlus(const Eigen::VectorXd &dx) {
  auto zipped = std::views::zip(_rotations, dx);
  std::ranges::for_each(zipped, [](const auto &rotations_zipped) {
    auto &[R, dtheta] = rotations_zipped;
    R = Eigen::Rotation2Dd(dtheta) * R;
  });
}

double State::distance(const State &other) const {
  return std::transform_reduce(
      this->_rotations.cbegin(), this->_rotations.cend(),
      other._rotations.cbegin(), 0.0, std::plus<double>(),
      [&](const Eigen::Rotation2Dd &R1, const Eigen::Rotation2Dd &R2) {
        return std::abs(R1.smallestPositiveAngle() -
                        R2.smallestPositiveAngle());
      });
}

// ---------- OPERATORS ----------
std::ostream &operator<<(std::ostream &os, const State &state) {
  std::ranges::for_each(state._rotations, [&](const Eigen::Rotation2Dd &R) {
    // Convert to degrees
    os << R.smallestPositiveAngle() * (180 / pi) << " ";
  });
  return os;
}

// ---------- STATIC METHODS ----------
std::tuple<State, std::vector<double>>
State::generateStateAndMeasurements(const int state_size) {
  // Initialize random number generator
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> generator(0.0, 2 * pi);

  // Initialize vector of angles and measurements
  std::vector<double> angles(state_size, 0.0);
  std::vector<double> measurements(state_size, 0.0);

  // Generate random angles
  std::ranges::for_each(angles, [&](double &angle) { angle = generator(mt); });
  angles[0] = 0.0; // The first at the origin

  // Generate the measurements
  for (int i = 0; i < state_size; i++) {
    const double &from_angle = angles[i];
    const double &to_angle = angles[(i + 1) % state_size];
    measurements[i] = (Eigen::Rotation2Dd(from_angle).inverse() *
                       Eigen::Rotation2Dd(to_angle))
                          .angle();
  }

  // Return the state generated from these angles
  return {State(angles), measurements};
}

} // namespace optilib
