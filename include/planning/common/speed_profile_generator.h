#pragma once

#include <utility>
#include <vector>

#include "autoagric/common/pnc_point.pb.h"
#include "planning/common/ego_info.h"
#include "planning/common/speed/speed_data.h"

namespace autoagric {
namespace planning {
class SpeedProfileGenerator {
 public:
  SpeedProfileGenerator() = default;
  static SpeedData GenerateFallbackSpeed(const EgoInfo* ego_info,
                                         const double stop_distance = 0.0);

  static void FillEnoughSpeedPoints(SpeedData* const speed_data);

  static SpeedData GenerateFixedDistanceCreepProfile(const double distance,
                                                     const double max_speed);

 private:
  static SpeedData GenerateStopProfile(const double init_speed,
                                       const double init_acc);
};

}  // namespace planning
}  // namespace autoagric