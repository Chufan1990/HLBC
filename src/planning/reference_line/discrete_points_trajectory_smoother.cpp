#include "planning/reference_line/discrete_points_trajectory_smoother.h"

#include <algorithm>

#include "common/macro.h"
#include "planning/math/discrete_points_math.h"
#include "planning/math/discretized_points_smoothing/cos_theta_smoother.h"

namespace autoagric {
namespace planning {

using common::TrajectoryPoint;

DiscretePointsTrajectorySmoother::DiscretePointsTrajectorySmoother(
    const TrajectorySmootherConfig& config)
    : TrajectorySmoother(config) {}

bool DiscretePointsTrajectorySmoother::Smooth(
    const ADCTrajectory& raw_trajectory,
    ADCTrajectory* const ptr_smoothed_trajectory) {
  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<double> anchorpoints_lateralbound;

  for (const auto& anchor_point : anchor_points_) {
    raw_point2d.emplace_back(anchor_point.path_point.x(),
                             anchor_point.path_point.y());

    anchorpoints_lateralbound.emplace_back(
        anchor_point.enforced ? 0.0 : anchor_point.lateral_bound);
  }

  NormalizePoints(&raw_point2d);

  bool status = false;

  const auto& smoothing_method = config_.discrete_points().smoothing_method();

  std::vector<std::pair<double, double>> smoothed_point2d;

  //   switch (smoothing_method) {
  //     case DiscretePointSmootherConfig::COS_THETA_SMOOTHING:
  //       status = CosThetaSmooth(raw_point2d, anchorpoints_lateralbound,
  //                               &smoothed_point2d);
  //       break;
  //     case DiscretePointSmootherConfig::FEM_POS_DEVIATION_SMOOTHING:
  //       status = FemPosSmooth(raw_point2d, anchorpoints_lateralbound,
  //                             &smoothed_point2d);
  //       break;
  //     default:
  //       AERROR("", "smoother type not defined");
  //       break;
  //   }

  /**
   * @note force to use costheta smoother
   */
  status =
      CosThetaSmooth(raw_point2d, anchorpoints_lateralbound, &smoothed_point2d);

  if (!status) {
    AERROR("", "discrete_points reference line smoother failed");
    return false;
  }

  DeNormalizePoints(&smoothed_point2d);

  std::vector<TrajectoryPoint> ref_points;
  GenerateTrajectoryPointProfile(raw_trajectory, smoothed_point2d, &ref_points);

  // ADCTrajectory::RemoveDuplicates(&ref_points);

  if (ref_points.size() < 2) {
    AERROR("", "fail to generate smoothed reference line.");
    return false;
  }

  ADCTrajectory smoothed_trajectory;

  smoothed_trajectory.mutable_header()->CopyFrom(raw_trajectory.header());

  for (std::size_t i = 0; i < ref_points.size(); i++)
    smoothed_trajectory.add_trajectory_point()->CopyFrom(ref_points[i]);

  *ptr_smoothed_trajectory = smoothed_trajectory;

  return true;
}

bool DiscretePointsTrajectorySmoother::CosThetaSmooth(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  const auto& cos_theta_config =
      config_.discrete_points().cos_theta_smoothing();

  CosThetaSmoother smoother(cos_theta_config);

  // box contraints on pos are used in cos theta smoother, thus shrink the
  // bounds by 1.0 / sqrt(2.0)
  std::vector<double> box_bounds = bounds;
  const double box_ratio = 1.0 / std::sqrt(2.0);
  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  std::vector<double> opt_x;
  std::vector<double> opt_y;
  bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

  if (!status) {
    AERROR("", "costheta reference line smoothing failed");
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    AERROR("", "return by costheta smoother is wrong. size smaller than 2");
    return false;
  }

  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

  std::size_t point_size = opt_x.size();
  for (std::size_t i = 0; i < point_size; i++) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

void DiscretePointsTrajectorySmoother::SetAnchorPoints(
    const std::vector<AnchorPoint>& anchor_points) {
  CHECK_GT(anchor_points.size(), 1U);
  anchor_points_ = anchor_points;
}

void DiscretePointsTrajectorySmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;

  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

void DiscretePointsTrajectorySmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

bool DiscretePointsTrajectorySmoother::GenerateTrajectoryPointProfile(
    const ADCTrajectory& raw_trajectory,
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<TrajectoryPoint>* trajectory_points) {
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;

  if (!DiscretePointsMath::ComputePathPofile(
          xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }

  std::size_t points_size = xy_points.size();

  for (std::size_t i = 0; i < points_size; i++) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(xy_points[i].first);
    point.mutable_path_point()->set_y(xy_points[i].second);
    point.mutable_path_point()->set_theta(headings[i]);
    point.mutable_path_point()->set_kappa(kappas[i]);
    point.mutable_path_point()->set_dkappa(dkappas[i]);
    point.mutable_path_point()->set_s(accumulated_s[i]);
    point.set_v(raw_trajectory.trajectory_point(i).v());
    point.set_relative_time(raw_trajectory.trajectory_point(i).relative_time());
    ADEBUG("", point.DebugString());
    trajectory_points->emplace_back(point);
  }
  return true;
}
}  // namespace planning
}  // namespace autoagric