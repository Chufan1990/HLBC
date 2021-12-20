/**
 * @file discrete_points_reference_line_smoother.h
 */

#pragma once

#include <utility>
#include <vector>

#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/planning/planning.pb.h"
#include "autoagric/planning/reference_line_smoother_config.pb.h"
#include "planning/reference_line/trajectory_smoother.h"

/**
 * @namespace autoagric::planning
 * @brief autoagric::planning
 */
namespace autoagric {
namespace planning {

class DiscretePointsTrajectorySmoother : public TrajectorySmoother {
 public:
  explicit DiscretePointsTrajectorySmoother(
      const TrajectorySmootherConfig& config);

  virtual ~DiscretePointsTrajectorySmoother() = default;

  bool Smooth(const ADCTrajectory& raw_trajectory,
              ADCTrajectory* const smoothed_trajectory) override;

  void SetAnchorPoints(const std::vector<AnchorPoint>&) override;

 private:
  bool CosThetaSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  //   bool FemPosSmooth(
  //       const std::vector<std::pair<double, double>>& raw_point2d,
  //       const std::vector<double>& bounds,
  //       std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  bool GenerateTrajectoryPointProfile(
      const ADCTrajectory& raw_trajectory,
      const std::vector<std::pair<double, double>>& xy_points,
      std::vector<common::TrajectoryPoint>* reference_points);

  std::vector<AnchorPoint> anchor_points_;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;
};

}  // namespace planning
}  // namespace autoagric