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

State::State(std::vector<Eigen::Rotation2Dd> &&rotations) {
  _rotations = std::move(rotations);
}

State::State(const size_t size) {
  _rotations = std::vector<Eigen::Rotation2Dd>(size, Eigen::Rotation2Dd(0.0));
}

// ---------- METHODS ----------
State State::boxPlus(const Eigen::VectorXd &dx) const {
  State new_state(this->size());
  auto zipped = std::views::zip(new_state._rotations, _rotations, dx);
  std::ranges::for_each(zipped, [](const auto &rotations_zipped) {
    auto &[R_new, R, dtheta] = rotations_zipped;
    R_new = Eigen::Rotation2Dd(dtheta) * R;
  });
  return new_state;
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
double State::norm() const {
  return std::sqrt(
      std::accumulate(_rotations.cbegin(), _rotations.cend(), 0.0,
                      [&](const double &val, const Eigen::Rotation2Dd &R) {
                        return val + R.smallestAngle() * R.smallestAngle();
                      }));
}

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
  // Initialize random number generator from 0 to 360 degrees
  /* std::random_device rd; */
  std::mt19937 mt(45); // TODO: fixed seed
  std::uniform_real_distribution<double> angles_generator(0.0, 2 * pi);
  std::uniform_int_distribution<int> ids_generator(0, state_size - 1);

  // Initialize vector of angles and measurements
  std::vector<Eigen::Rotation2Dd> rotations(state_size,
                                            Eigen::Rotation2Dd(0.0));
  std::vector<Measurement> measurements;
  measurements.reserve((state_size - 1) + n_closures);

  // Generate random rotations: STATE
  std::ranges::for_each(
      rotations.begin() + 1, rotations.end(), [&](Eigen::Rotation2Dd &R) {
        R = std::move(Eigen::Rotation2Dd(angles_generator(mt)));
      });

  // Generate pose to pose MEASUREMENTS
  std::ranges::for_each(std::views::enumerate(rotations).cbegin(),
                        std::views::enumerate(rotations).cend() - 1,
                        [&](const auto &idx_R) {
                          const auto &[idx, R] = idx_R;
                          measurements.emplace_back(
                              R.inverse() * rotations[idx + 1], idx, idx + 1);
                        });

  // Generate the CLOSURES
  std::vector<int> state_ids(state_size, 0);
  std::iota(state_ids.begin(), state_ids.end(), 0);
  std::shuffle(state_ids.begin(), state_ids.end(), mt);
  std::ranges::for_each(
      state_ids.cbegin(), state_ids.cbegin() + n_closures, [&](const int from) {
        int to = ids_generator(mt);
        // Avoid self-measurements
        while (from == to) {
          to = ids_generator(mt);
        }
        measurements.emplace_back(rotations[from].inverse() * rotations[to],
                                  from, to);
      });

  // Return the state and measurements
  return {State(std::move(rotations)), measurements};
}

} // namespace optilib
