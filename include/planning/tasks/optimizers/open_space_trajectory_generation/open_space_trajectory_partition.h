#pragma once

#include <string>

#include "common/status/status.h"
#include "planning/common/trajectory/discretized_trajectory.h"

namespace autoagric {
namespace planning {

class OpenSpaceTrajectoryPartition {
 public:
  OpenSpaceTrajectoryPartition() = default;

 private:
  common::Status Process();

  OpenSpaceTrajectoryPartitionConfig open_space_trajectory_partition_config_;
  double heading_search_range_ = 0.0;
  double heading_track_range_ = 0.0;
  double distance_search_range_ = 0.0;
  double heading_offset_to_midpoint_ = 0.0;
  double lateral_offset_to_midpoint_ = 0.0;
  double longitudinal_offset_to_midpoint_ = 0.0;
  double vehicle_box_iou_threshold_to_midpoint_ = 0.0;
  double linear_velocity_threshold_on_ego_ = 0.0;
}

}  // namespace planning
}  // namespace autoagric