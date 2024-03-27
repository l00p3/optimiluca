#include "Lumath.hpp"
#include <algorithm>
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
  std::ranges::for_each(std::views::enumerate(this->_T_matrices),
                        [&](const auto &idx_T) {
                          const auto &[idx, T] = idx_T;
                          new_state(idx) = v2T(dx.segment<6>(idx * 6)) * T;
                        });
  return new_state;
}

double State::distance(const State &other) const {
  return std::transform_reduce(
      this->_T_matrices.cbegin(), this->_T_matrices.cend(),
      other._T_matrices.cbegin(), 0.0, std::plus<double>(),
      [&](const Eigen::Matrix4d &T1, const Eigen::Matrix4d &T2) {
        return flatten(T1 - T2).squaredNorm();
      });
}

// ---------- OPERATORS ----------
double State::norm() const {
  return std::sqrt(
      std::accumulate(_T_matrices.cbegin(), _T_matrices.cend(), 0.0,
                      [&](const double &val, const Eigen::Matrix4d &T) {
                        return val + T.norm();
                      }));
}

// ---------- STATIC METHODS ----------
std::tuple<State, std::vector<Measurement>>
State::generateStateAndMeasurements(const int state_size,
                                    const int n_closures) {
  // Initialize random number generator from 0 to 360 degrees
  /* std::random_device rd; */
  std::mt19937 mt(45); // TODO: fixed seed
  std::uniform_real_distribution<double> vector_generator(-10.0, 10.0);
  std::uniform_int_distribution<int> ids_generator(0, state_size - 1);

  // Initialize vector of angles and measurements
  std::vector<Eigen::Matrix4d> T_matrices(state_size,
                                          Eigen::Matrix4d::Identity());
  std::vector<Measurement> measurements;
  measurements.reserve((state_size - 1) + n_closures);

  // Rotation generator (3D axis angle vector repesentation)
  auto rotation_generator = [&]() {
    const Eigen::Vector3d omega_vector =
        2 * pi * Eigen::Vector3d::Random().array() - pi;
    const double angle = omega_vector.norm();
    return Eigen::Matrix3d(Eigen::AngleAxisd(angle, omega_vector.normalized()));
  };

  // Vector generator
  auto vector3d_generator = [&]() {
    return Eigen::Vector3d(
        {vector_generator(mt), vector_generator(mt), vector_generator(mt)});
  };

  // Generate random rotations: STATE
  std::ranges::for_each(T_matrices.begin() + 1, T_matrices.end(),
                        [&](Eigen::Matrix4d &T) {
                          // Generate rotation
                          T.block<3, 3>(0, 0) = rotation_generator();
                          // Generate translation
                          T.block<3, 1>(0, 3) = vector3d_generator();
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
