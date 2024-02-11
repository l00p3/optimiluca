#include "State.hpp"

namespace optilib {

State::State(const double &r1, const double &r2, const double &r3,
             const double &r4)
    : _rotations(4, Rot2D::Identity()) {
  // Initialize the elements of the state
  this->_rotations[0] = Eigen::Rotation2Dd(r1);
  this->_rotations[1] = Eigen::Rotation2Dd(r2);
  this->_rotations[2] = Eigen::Rotation2Dd(r3);
  this->_rotations[3] = Eigen::Rotation2Dd(r4);
}

// ---------- METHODS ----------
size_t State::size() const { return this->_rotations.size(); }

Rot2D State::get_rotation(const int &idx) const {
  assert(idx >= 0 && idx < this->size() &&
         "Invalid index of rotation in State");
  return this->_rotations[idx];
}

// ---------- OPERATORS ----------
State State::boxPlus(const double &dx) const {
  /* return State(Eigen::Rotation2Dd(dx) * this->_rotations[0], */
  /*              Eigen::Rotation2Dd(dx) * this->_rotations[1], */
  /*              Eigen::Rotation2Dd(dx) * this->_rotations[2], */
  /*              Eigen::Rotation2Dd(dx) * this->_rotations[3]); */
}

std::ostream &operator<<(std::ostream &os, const State &state) {
  for (const auto &rot : state._rotations) {
    os << rot.angle() << " ";
  }
  os << std::endl;
  return os;
}

} // namespace optilib
