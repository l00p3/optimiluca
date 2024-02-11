#include <iostream>

#include "State.hpp"

namespace optilib {

State::State(const double &r1, const double &r2, const double &r3,
             const double &r4)
    : _rotations(4, Rot2D::Identity()) {
  // Initialize the elements of the state
  this->_rotations[0] = Rot2D(r1);
  this->_rotations[1] = Rot2D(r2);
  this->_rotations[2] = Rot2D(r3);
  this->_rotations[3] = Rot2D(r4);
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
