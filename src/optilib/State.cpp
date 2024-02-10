#include "State.hpp"

namespace optilib {

State::State(const double &r1, const double &r2, const double &r3,
             const double &r4)
    : _rotations(4, Eigen::Rotation2D<double>::Identity()) {
  // Initialize the elements of the state
  this->_rotations[0] = Eigen::Rotation2Dd(r1);
  this->_rotations[1] = Eigen::Rotation2Dd(r2);
  this->_rotations[2] = Eigen::Rotation2Dd(r3);
  this->_rotations[3] = Eigen::Rotation2Dd(r4);
}

// ---------- METHODS ----------
size_t State::size() const { return this->_rotations.size(); }

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
