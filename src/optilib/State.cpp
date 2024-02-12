#include <iostream>

#include "State.hpp"

namespace optilib {

State::State(const std::vector<double> &angles)
    : _rotations(4, Rot2D::Identity()) {
  // Initialize the elements of the state
  assert(angles.size() == 4 &&
         "We are dealing exactly with 4 angles in the state.");
  for (int angle_idx = 0; angle_idx < angles.size(); angle_idx++) {
    this->_rotations[angle_idx] = Rot2D(angles[angle_idx]);
  }
}

// ---------- METHODS ----------
size_t State::size() const { return this->_rotations.size(); }

Rot2D State::get_rotation(const int &idx) const {
  assert(idx >= 0 && idx < this->size() &&
         "Invalid index of rotation in State");
  return this->_rotations[idx];
}

// ---------- OPERATORS ----------
void State::boxPlus(const Eigen::Vector3d &dx) {
  int current_state_idx = 1;
  for (const auto &dx_i : dx) {
    this->_rotations[current_state_idx++] =
        Rot2D(Rot2D(dx_i).toRotationMatrix().transpose() *
              this->_rotations[current_state_idx++].toRotationMatrix());
  }
}

std::ostream &operator<<(std::ostream &os, const State &state) {
  for (const auto &rot : state._rotations) {
    os << rot.angle() << " ";
  }
  return os;
}

} // namespace optilib
