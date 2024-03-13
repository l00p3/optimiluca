#pragma once

#include "State.hpp"

namespace optilib {

class Solver {

public:
  virtual std::vector<double>
  solve(State &state, const std::vector<Measurement> &measurements,
        const int n_iters, const int verbose_level) = 0;
};

class GSSolver : public Solver {
public:
  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const int n_iters = 10,
                            const int verbose_level = 1);
};

class DLSolver : public Solver {
public:
  std::vector<double> solve(State &state,
                            const std::vector<Measurement> &measurements,
                            const int n_iters = 10,
                            const int verbose_level = 1);
};

} // namespace optilib
