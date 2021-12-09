#pragma once

#include <memory>

#include "autoagric/planning/reference_line_smoother_config.pb.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/macro.h"
#include "control/common/msg_to_proto.h"
#include "control/controller/controller_agent.h"
#include "planning/reference_line/trajectory_smoother.h"

/**
 * @namespace autoagric::control
 * @brief autoagric::control
 */
namespace autoagric {
namespace control {

class ControllerInterface {
 public:
  ControllerInterface() = default;

  virtual ~ControllerInterface() = default;

  bool Init();

  autoagric::common::Status ComputeControlCommand(
      const localization::LocalizationEstimate* localization,
      const canbus::Chassis* chassis, const planning::ADCTrajectory* trajectory,
      control::ControlCommand* cmd);

  template <typename L, typename C, typename T, typename F>
  bool ComputeControlCommand(const L& localization, const C& chassis,
                             const T& trajectory, const F& feedback,
                             double& ret) {
    GetProtoFromMsg(trajectory, local_view_.mutable_trajectory());

    GetProtoFromMsg(feedback, local_view_.mutable_chassis());

    GetProtoFromMsg(localization, chassis, local_view_.mutable_localization());

    std::vector<planning::AnchorPoint> anchor_points;

    for (auto& point : local_view_.trajectory().trajectory_point()) {
      anchor_points.emplace_back(
          point.path_point(),
          trajectory_smoother_conf_.max_lateral_boundary_bound(),
          trajectory_smoother_conf_.longitudinal_boundary_bound());
    }

    anchor_points.front().enforced = true;
    anchor_points.back().enforced = true;

    smoother_->SetAnchorPoints(anchor_points);
    smoother_->Smooth(local_view_.trajectory(),
                      local_view_.mutable_trajectory());

    ADEBUG(local_view_.trajectory().DebugString());

    auto status = CheckInput(&local_view_);

    if (!status.ok()) {
      AERROR_EVERY(100,
                   "Control input data failed: " << status.error_message());
    } else {
      autoagric::common::Status status_ts = CheckTimestamp(local_view_);
      if (!status_ts.ok()) {
        AERROR("Input messages timeout");
        status = status_ts;
      }
    }

    status = controller_agent_.ComputeControlCommand(
        &local_view_.localization(), &local_view_.chassis(),
        &local_view_.trajectory(), &control_command_);
    if (!status.ok()) {
      AERROR(status.error_message());
      return false;
    }
    auto vehicle_param =
        common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
    ret = control_command_.steering_target() / 100.0 *
          vehicle_param.max_steer_angle();

    return true;
  }

  planning::ADCTrajectory trajectory() const {
    return local_view_.trajectory();
  }

 private:
  ControllerAgent controller_agent_;

  autoagric::common::Status CheckInput(LocalView* local_view);
  autoagric::common::Status CheckTimestamp(const LocalView& local_view);

  LocalView local_view_;
  ControlCommand control_command_;

  std::shared_ptr<DependencyInjector> injector_;
  std::unique_ptr<planning::TrajectorySmoother> smoother_;
  ControlConf control_conf_;
  planning::TrajectorySmootherConfig trajectory_smoother_conf_;
};

}  // namespace control
}  // namespace autoagric