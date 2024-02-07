#include <iostream>

#include <optilib/Solver.hpp>
#include <vector>

using namespace optilib;

int main() {
  // Initialization
  auto solver = Solver();

  // Define the state
  std::vector<float>
      state; // TODO: this should be a vector of rotation matrices

  std::cout << "Funziono" << std::endl;
}
