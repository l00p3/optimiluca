#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <ranges>

#include <State.hpp>
#include <vector>

using namespace std::numbers;

namespace optilib {
State::State(const std::vector<double> &angles) {
  _rotations.reserve(angles.size());
  std::ranges::for_each(
      angles, [&](const double &theta) { _rotations.emplace_back(theta); });
}

State::State(const std::vector<Eigen::Rotation2Dd> &&rotations) {
  _rotations = std::move(rotations);
}

State::State(const size_t size) {
  _rotations = std::vector<Eigen::Rotation2Dd>(size, Eigen::Rotation2Dd(0.0));
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
  // L1-norm of the angles distance
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
std::tuple<State, std::vector<Measurement>>
State::generateStateAndMeasurements(const int state_size,
                                    const int n_closures) {
  // Initialize random number generator
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> generator(0.0,
                                                   2 * pi); // 0 to 360 degrees

  // Initialize list of indices for loops
  std::vector<int> state_ids = std::vector<int>(state_size);
  std::iota(state_ids.begin(), state_ids.end(), 0);

  // Initialize vector of angles and measurements
  std::vector<Eigen::Rotation2Dd> rotations;
  rotations.reserve(state_size);
  std::vector<Measurement> measurements;
  measurements.reserve((state_size - 1) + n_closures);

  // Generate the state
  std::ranges::for_each(state_ids, [&](const int state_idx) {
    rotations.emplace_back(Eigen::Rotation2Dd(generator(mt)));
  });
  rotations[0] = Eigen::Rotation2Dd(0.0); // The first at the origin
  State state = State(std::move(rotations));

  // Generate the measurements
  std::ranges::for_each(state_ids, [&](const int state_idx) {
    int from = state_idx;
    int to = (state_idx + 1) % state_size;
    measurements.emplace_back(state(from).inverse() * state(to), from, to);
  });

  // Generate the closures
  // TODO

  // Return the state generated from these angles
  return {state, measurements};
}

} // namespace optilib
