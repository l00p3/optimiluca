#pragma once

#include "State.hpp"

namespace optilib {

class Solver {

public:
  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const int n_iters = 10,
                            const int verbose_level = 1);
};

} // namespace optilib
