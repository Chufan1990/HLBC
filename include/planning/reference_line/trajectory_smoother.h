/**
 * @file
 */

#pragma once

#include <vector>

#include "hlbc/proto/pnc_point.pb.h"
#include "hlbc/proto/reference_line_smoother_config.pb.h"
// #include "planning/reference_line/reference_line.h"

/**
 * @namespace autoagric::planning
 * @brief autoagric::planning
 */
namespace autoagric {
namespace planning {

struct AnchorPoint {
  common::PathPoint path_point;
  double lateral_bound = 0.0;
  double longitudinal_bound = 0.0;
  // enforce smoother to strictly follow this reference point
  bool enforced = false;
};

class TrajectorySmoother {
 public:
  explicit TrajectorySmoother(const TrajectorySmootherConfig& config)
      : config_(config) {}

  virtual ~TrajectorySmoother() = default;
  /**
   * @brief smoothing constraints
   */
  virtual void SetAnchorPoints(
      const std::vector<AnchorPoint>& achor_points) = 0;

  /**
   * @brief smooth a given reference line
   */
  virtual bool Smooth(const ADCTrajectory&, ADCTrajectory* const) = 0;

 protected:
  TrajectorySmootherConfig config_;
};
}  // namespace planning
}  // namespace autoagric