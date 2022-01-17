#pragma once

#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning/common/ego_info.h"

namespace autoagric {
namespace planning {
class DependencyInjector final {
 public:
  DependencyInjector() = default;
  ~DependencyInjector() = default;

  EgoInfo* ego_info() { return &ego_info_; }

  common::VehicleStateProvider* vehicle_state() { return &vehicle_state_; }

 private:
  EgoInfo ego_info_;
  common::VehicleStateProvider vehicle_state_;
};
}  // namespace planning
}  // namespace autoagric