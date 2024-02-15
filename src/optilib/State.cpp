#include <iostream>
#include <ranges>

#include <State.hpp>

namespace optilib {

State::State(const std::vector<double> &angles) {
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

// ---------- OPERATORS ----------
std::ostream &operator<<(std::ostream &os, const State &state) {
  for (const auto &rot : state._rotations) {
    const double angle_degrees = rot.angle() * (180 / pi);
    os << angle_degrees << " ";
  }
  return os;
}

} // namespace optilib
