#include "planning/common/speed_profile_generator.h"

#include <algorithm>
#include <utility>

#include "common/configs/vehicle_config_helper.h"
#include "common/macro.h"
#include "planning/common/planning_gflags.h"

namespace autoagric {
namespace planning {
using autoagric::common::SpeedPoint;

SpeedData SpeedProfileGenerator::GenerateFallbaclSpeed(
    const EgoInfo* ego_info, const double stop_distance) {
  AINFO("Fallback using piecewise jerk speed");
  const double init_v = ego_info->start_point().v();
  const double inti_a = ego_info->start_point().a();
  AWARN("init_v = " << init_v << ", init_a = " << init_a);
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  if (init_v <= 0.0 && init_a <= 0.0) {
    AWARN("Already stopped! Nothing to do in GenerateFallbackSpeed()");
    SpeedData speed_data;
    speed_data.AppendSpeedPoint(0.0, 0.0, 0.0, 0.0, 0.0);
    FillEnoughSpeedPoints(&speed_data);
    return speed_data;
  }

  std::array<double, 3> init_s = {0.0, init_v, init_a};

  double delta_t = FLAGS_fallback_time_unit;
  double total_time = FLAGS_fallback_total_time;
  const size_t num_of_knots = static_cast<size_t>(total_time / delta_t) + 1;

  




}
}  // namespace planning
}  // namespace autoagric