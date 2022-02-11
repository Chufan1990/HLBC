#pragma once

#include <utility>
#include <vector>

#include "osqp.h"

namespace autoagric {
namespace planning {

class FemPosDeviationSqpOsqpInterface {
  FemPosDeviationSqpOsqpInterface() = default;

  virtual ~FemPosDeviationSqpOsqpInterface() = default;

 private:
  // Init states and constraints
  std::vector<std::pair<double, double>> ref_points_;
  std::vector<double> bounds_around_refs_;
  double curvature_constraint_ = 0.2;

  // Weight in optimization cost function
  double weight_fem_pos_deviation_ = 1e5;
  double weight_path_length_ = 1.0;
  double weight_ref_deviation_ = 1.0;
  double weight_curvature_constraint_slack_var_ = 1e5;

  // Settings of osqp
  int max_iter_ = 4000;
  double time_limit_ = 0.0;
  bool verbose_ = false;
  bool scaled_termination_ = true;
  bool warm_start_ = true;

  // Settings of sqp
  int sqp_pen_max_iter_ = 100;
  double sqp_ftol_ = 1e-2;
  int sqp_sub_max_iter_ = 100;
  double sqp_ctol_ = 1e-2;

  // Optimization problem definitions
  int num_of_points_ = 0;
  int num_of_pos_variables_ = 0;
  int num_of_slack_variables_ = 0;
  int num_of_variables_ = 0;
  int num_of_variables_constraints_ = 0;
  int num_of_curvature_constraints_ = 0;
  int num_of_constraints_ = 0;

  // Optimized result
  std::vector<std::pair<double, double>> opt_xy_;
  std::vector<double> slack_;
  double average_interval_length_ = 0.0;
};

}  // namespace planning
}  // namespace autoagric