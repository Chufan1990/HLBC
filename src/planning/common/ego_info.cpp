#include "planning/common/ego_info.h"

#include "common/configs/vehicle_config_helper.h"
#include "common/macro.h"

namespace autoagric {
namespace planning {
using common::math::Box2d;
using common::math::Vec2d;

EgoInfo::EgoInfo() {
  ego_vehicle_config_ = common::VehicleConfigHelper::GetConfig();
}

bool EgoInfo::Update(const common::TrajectoryPoint& start_point,
                     const common::VehicleState& vehicle_state) {
  set_start_point(start_point);
  set_vehicle_state(vehicle_state);
  CalculateEgoBox(vehicle_state);
  return true;
}

void EgoInfo::CalculateEgoBox(const common::VehicleState& vehicle_state) {
  const auto& param = ego_vehicle_config_.vehicle_param();
  ADEBUG("param " << param.DebugString());

  Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);

  Vec2d position(vehicle_state.x(), vehicle_state.y());

  Vec2d center(position + vec_to_center.rotate(vehicle_state.heading()));

  ego_box_ =
      Box2d(center, vehicle_state.heading(), param.length(), param.width());
}

void EgoInfo::Clear() {
  start_point_.Clear();
  vehicle_state_.Clear();
  front_clear_distance_ = FLAGS_default_front_clear_distance;
}

}  // namespace planning
}  // namespace autoagric