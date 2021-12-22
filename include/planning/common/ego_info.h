#pragma once

#include <vector>

#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/common/vehicle_config.pb.h"
#include "autoagric/common/vehicle_state.pb.h"
#include "common/macro.h"
#include "common/math/box2d.h"
#include "planning/common/planning_gflags.h"

namespace autoagric {
namespace planning {

class EgoInfo {
 public:
  EgoInfo();

  ~EgoInfo() = default;

  bool Update(const common::TrajectoryPoint& start_point,
              const common::VehicleState& vehicle_state);

  void Clear();

  common::TrajectoryPoint start_point() const {
    return start_point_;
  }

  common::VehicleState vehicle_state() const {
    return vehicle_state_;
  }

  double front_clear_distance() const { return front_clear_distance_; }

  common::math::Box2d ego_box() const { return ego_box_; }

 private:
  void set_vehicle_state(const common::VehicleState& vehicle_state) {
    vehicle_state_ = vehicle_state;
  }

  void set_start_point(const common::TrajectoryPoint& start_point) {
    start_point_ = start_point;
    const auto& param = ego_vehicle_config_.vehicle_param();
    start_point_.set_a(
        std::fmax(std::fmin(start_point_.a(), param.max_acceleration()),
                  param.max_deceleration()));
  }

  void CalculateEgoBox(const common::VehicleState& vehicle_state);

  common::TrajectoryPoint start_point_;

  common::VehicleState vehicle_state_;

  double front_clear_distance_ = FLAGS_default_front_clear_distance;

  common::VehicleConfig ego_vehicle_config_;

  common::math::Box2d ego_box_;
};

}  // namespace planning
}  // namespace autoagric