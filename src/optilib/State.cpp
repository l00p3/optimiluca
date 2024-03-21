#include "Lumath.hpp"
#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <ranges>

#include <State.hpp>
#include <vector>

using namespace std::numbers;

namespace optilib {
State::State(std::vector<Eigen::Matrix4d> &&T_matrices) {
  _T_matrices = std::move(T_matrices);
}

State::State(const size_t size) {
  _T_matrices = std::vector<Eigen::Matrix4d>(size, Eigen::Matrix4d::Identity());
}

// ---------- METHODS ----------
State State::boxPlus(const Eigen::VectorXd &dx) const {
  State new_state(this->size());
  std::ranges::for_each(
      std::views::enumerate(this->_T_matrices).cbegin(),
      std::views::enumerate(this->_T_matrices).cend(), [&](const auto &idx_T) {
        const auto &[idx, T] = idx_T;
        new_state._T_matrices[idx] = v2T(dx.block<6, 1>(idx * 6, 0)) * T;
      });
  return new_state;
}

double State::distance(const State &other) const {
  // L1-norm of the angles distance
  /* return std::transform_reduce( */
  /*     this->_rotations.cbegin(), this->_rotations.cend(), */
  /*     other._rotations.cbegin(), 0.0, std::plus<double>(), */
  /*     [&](const Eigen::Rotation2Dd &R1, const Eigen::Rotation2Dd &R2) { */
  /*       return std::abs(R1.smallestPositiveAngle() - */
  /*                       R2.smallestPositiveAngle()); */
  /*     }); */
  return 123456; // TODO
}

// ---------- OPERATORS ----------
double State::norm() const {
  /* return std::sqrt( */
  /*     std::accumulate(_rotations.cbegin(), _rotations.cend(), 0.0, */
  /*                     [&](const double &val, const Eigen::Rotation2Dd &R) {
   */
  /*                       return val + R.smallestAngle() * R.smallestAngle();
   */
  /*                     })); */
  return 0.0; // TODO
}

// ---------- STATIC METHODS ----------
std::tuple<State, std::vector<Measurement>>
State::generateStateAndMeasurements(const int state_size,
                                    const int n_closures) {
  // Initialize random number generator from 0 to 360 degrees
  /* std::random_device rd; */
  std::mt19937 mt(45); // TODO: fixed seed
  std::uniform_real_distribution<double> angles_generator(0.0, 2 * pi);
  std::normal_distribution<double> vector_generator(0.0, 1.0);
  std::uniform_int_distribution<int> ids_generator(0, state_size - 1);

  // Initialize vector of angles and measurements
  std::vector<Eigen::Matrix4d> T_matrices(state_size,
                                          Eigen::Matrix4d::Identity());
  std::vector<Measurement> measurements;
  measurements.reserve((state_size - 1) + n_closures);

  // Rotation generator
  auto rotation_generator = [&]() {
    Eigen::Vector3d rotation_axis(
        {vector_generator(mt), vector_generator(mt), vector_generator(mt)});
    rotation_axis.normalize();
    return Eigen::Matrix3d(
        Eigen::AngleAxisd(angles_generator(mt), rotation_axis));
  };

  // Generate random rotations: STATE
  std::ranges::for_each(T_matrices.begin() + 1, T_matrices.end(),
                        [&](Eigen::Matrix4d &T) {
                          // Generate rotation
                          T.block<3, 3>(0, 0) = rotation_generator();
                          // Generate translation // TODO
                          T.block<3, 1>(0, 3) = Eigen::Vector3d::Zero();
                        });

  // Generate pose to pose MEASUREMENTS
  std::ranges::for_each(std::views::enumerate(T_matrices).cbegin(),
                        std::views::enumerate(T_matrices).cend() - 1,
                        [&](const auto &idx_T) {
                          const auto &[idx, T] = idx_T;
                          measurements.emplace_back(
                              T_inverse(T) * T_matrices[idx + 1], idx, idx + 1);
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
        measurements.emplace_back(T_inverse(T_matrices[from]) * T_matrices[to],
                                  from, to);
      });

  // Return the state and measurements
  return {State(std::move(T_matrices)), measurements};
}

} // namespace optilib
