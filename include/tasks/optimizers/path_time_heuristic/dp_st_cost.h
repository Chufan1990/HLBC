#pragma once

#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/planning/st_drivable_boundary.pb.h"
#include "autoagric/planning/task_config.pb.h"
#include "planning/common/obstacle.h"

namespace autoagric {
namespace planning {

class DpStCost {
 public:
  DpStCost(const DpStSpeedOptimizerConfig& config, const double total_t,
           const double total_s, const std::vector<const Obstacle*>& obstacles,
           const STDrivableBoundary& st_drivable_boundary,
           const common::TrajectoryPoint& init_point);
};

}  // namespace planning
}  // namespace autoagric