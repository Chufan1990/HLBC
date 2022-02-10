#include "planning/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"

#include "common/macro.h"
#include "planning/math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.h"

namespace autoagric {
namespace planning {

FemPosDeviationSmoother::FemPosDeviationSmoother(
    const FemPosDeviationSmootherConfig& config)
    : config_(config) {
  AINFO("Using FemPosDeviationSmootherConfig: " << config.DebugString());
}

bool FemPosDeviationSmoother::Solve(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  CHECK_NOTNULL(opt_x);
  CHECK_NOTNULL(opt_y);
  if (config_.apply_curvature_constraint()) {
    if (config_.use_sqp()) {
      return SqpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
    } else {
      return NlpWithIpopt(raw_point2d, bounds, opt_x, opt_y);
    }
  } else {
    return QpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
  }
  return true;
}

bool FemPosDeviationSmoother::SqpWithOsqp(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  CHECK_NOTNULL(opt_x);
  CHECK_NOTNULL(opt_y);
  return true;
}

bool FemPosDeviationSmoother::NlpWithIpopt(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  CHECK_NOTNULL(opt_x);
  CHECK_NOTNULL(opt_y);
  return true;
}

bool FemPosDeviationSmoother::QpWithOsqp(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  CHECK_NOTNULL(opt_x);
  CHECK_NOTNULL(opt_y);

  FemPosDeviationOsqpInterface solver;

  solver.set_weight_fem_pos_deviation(config_.weight_fem_pos_deviation());
  solver.set_weight_path_length(config_.weight_path_length());
  solver.set_weight_ref_deviation(config_.weight_ref_deviation());

  solver.set_max_iter(config_.max_iter());
  solver.set_time_limit(config_.time_limit());
  solver.set_verbose(config_.verbose());
  solver.set_scaled_termination(config_.scaled_termination());
  solver.set_warm_start(config_.warm_start());

  solver.set_ref_points(raw_point2d);
  solver.set_bounds_around_refs(bounds);

  if (!solver.Solve()) {
    return false;
  }

  *opt_x = solver.opt_x();
  *opt_y = solver.opt_y();

  return true;
}

}  // namespace planning
}  // namespace autoagric