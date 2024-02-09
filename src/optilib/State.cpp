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

State State::boxPlus(const double &dx) const {
  /* return State(Eigen::Rotation2Dd(dx) * this->_rotations[0], */
  /*              Eigen::Rotation2Dd(dx) * this->_rotations[1], */
  /*              Eigen::Rotation2Dd(dx) * this->_rotations[2], */
  /*              Eigen::Rotation2Dd(dx) * this->_rotations[3]); */
}

} // namespace optilib
