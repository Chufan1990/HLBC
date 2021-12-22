#pragma once

#include <vector>

#include <utility>

#include "planning/math/piecewise_jerk/piecewise_jerk_problem.h"

namespace autoagric {
namespace planning {

class PiecewiseJerkSpeedProblem : public PiecewiseJerkProblem {
 public:
  PiecewiseJerkSpeedProblem(const size_t num_of_knots, const double delta_s,
                            const std::array<double, 3>& x_init);

  virtual ~PiecewiseJerkSpeedProblem() = default;

  void set_dx_ref(const double weight_dx_ref, const double dx_ref);

  void set_penalty_dx(std::vector<double> penalty_dx);

 protected:
  // naming convention follows osqp solver.
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indice,
                       std::vector<c_int>* P_indptr) override;

  void CalculateOffset(std::vector<c_float>* q) override;

  OSQPSettings* SolverDefaultSettings() override;

  bool has_dx_ref_ = false;
  double weight_dx_ref_ = 0.0;
  double dx_ref_ = 0.0;

  std::vector<double> penalty_dx_;
};

}  // namespace planning
}  // namespace autoagric