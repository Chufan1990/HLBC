#pragma once

#include <memory>

#include "common/configs/vehicle_config_helper.h"
#include "common/macro.h"
#include "common/util/file.h"
#include "control/common/control_gflags.h"
#include "control/common/msg_to_proto.h"
#include "control/controller/controller_agent.h"
#include "hlbc/proto/error_code.pb.h"
/**
 * @namespace autoagric::control
 * @brief autoagric::control
 */
namespace autoagric {
namespace control {

using autoagric::common::ErrorCode;
using autoagric::common::Status;

template <typename L, typename C, typename T, typename F>
class ControllerInterface {
 public:
  ControllerInterface() = default;

  virtual ~ControllerInterface() = default;

  bool Init();

  bool ComputeControlCommand(const L& localization, const C& chassis,
                             const T& trajectory, const F& feedback,
                             double& ret);

 private:
  ControllerAgent controller_agent_;

  common::Status CheckInput(LocalView* local_view);
  common::Status CheckTimestamp(const LocalView& local_view);

  LocalView local_view_;
  ControlCommand control_command_;

  std::shared_ptr<DependencyInjector> injector_;
  ControlConf control_conf_;
};

template <typename L, typename C, typename T, typename F>
bool ControllerInterface<L, C, T, F>::Init() {
  injector_ = std::make_shared<DependencyInjector>();

  ACHECK(
      !common::util::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_),
      "controller/controller_agent.cpp, ControllerAgent::Init",
      "Unable to load control conf file: " << FLAGS_control_conf_file);

  AINFO("controller/controller_agent.cpp, ControllerAgent::Init",
        "Conf file: " << FLAGS_control_conf_file << " is loaded.");

  ADEBUG("controller/controller_agent.cpp, ControllerAgent::Init",
         "FLAGS_use_control_submodules: " << FLAGS_use_control_submodules);

  if (!controller_agent_.Init(injector_, &control_conf_).ok()) return false;
  return true;
}

template <typename L, typename C, typename T, typename F>
bool ControllerInterface<L, C, T, F>::ComputeControlCommand(
    const L& localization, const C& chassis, const T& trajectory,
    const F& feedback, double& ret) {
  GetProtoFromMsg(trajectory, local_view_.mutable_trajectory());

  GetProtoFromMsg(feedback, local_view_.mutable_chassis());

  GetProtoFromMsg(localization, chassis, local_view_.mutable_localization());

  UpdateTrajectoryPoint(&local_view_.localization(),
                        local_view_.mutable_trajectory());

  auto status = CheckInput(&local_view_);

  if (!status.ok()) {
    AERROR_EVERY(100,
                 "controller/controller_agent.cpp, "
                 "ControllerAgent::ComputeControlCommand",
                 "Control input data failed: " << status.error_message());
  } else {
    Status status_ts = CheckTimestamp(local_view_);
    if (!status_ts.ok()) {
      AERROR(
          "controller/controller_agent.cpp, "
          "ControllerAgent::ComputeControlCommand",
          "Input messages timeout");
      status = status_ts;
    }
  }

  status = controller_agent_.ComputeControlCommand(
      &local_view_.localization(), &local_view_.chassis(),
      &local_view_.trajectory(), &control_command_);
  if (!status.ok()) {
    AERROR("", status.error_message());
    return false;
  }
  auto vehicle_param =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  ret = control_command_.steering_target() / 100.0 *
        vehicle_param.max_steer_angle();

  return true;
}

template <typename L, typename C, typename T, typename F>
Status ControllerInterface<L, C, T, F>::CheckInput(LocalView* local_view) {
  ADEBUG("controller/controller_agent.cpp, ControllerAgent::CheckInput",
         "Received localization:"
             << local_view->localization().ShortDebugString());
  ADEBUG("controller/controller_agent.cpp, ControllerAgent::CheckInput",
         "Received chassis:" << local_view->chassis().ShortDebugString());

  if (local_view->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100,
                "controller/controller_agent.cpp, ControllerAgent::CheckInput",
                "planning has no trajectory point. ");

    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "planning has no trajectory point. planning_seq_num:" +
                      local_view->trajectory().header().sequence_num());
  }

  for (auto& trajectory_point :
       *local_view->mutable_trajectory()->mutable_trajectory_point()) {
    if (std::abs(trajectory_point.v()) <
            control_conf_.minimum_speed_resolution() &&
        std::abs(trajectory_point.a()) <
            control_conf_.max_acceleration_when_stopped()) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }

  auto status = injector_->vehicle_state()->Update(local_view->localization(),
                                                   local_view->chassis());

  return status;
}

template <typename L, typename C, typename T, typename F>
Status ControllerInterface<L, C, T, F>::CheckTimestamp(
    const LocalView& local_view) {
  if (!control_conf_.enable_input_timestamp_check() ||
      control_conf_.is_control_test_mode()) {
    ADEBUG("controller/controller_agent, ControllerAgent::CheckTimestamp",
           "Skip input timestamp check by gflags");
    return Status::OK();
  }

  double current_timestamp = ros::Time::now().toSec();

  double localization_diff =
      current_timestamp - local_view.localization().header().timestamp_sec();
  if (localization_diff > (control_conf_.max_localization_miss_num() *
                           control_conf_.localization_period())) {
    AERROR("controller/controller_agent, ControllerAgent::CheckTimestamp",
           "localization msg lost for " << std::setprecision(6)
                                        << localization_diff << "s");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "localization msg timeout");
  }

  double chassis_diff =
      current_timestamp - local_view.chassis().header().timestamp_sec();
  if (chassis_diff >
      (control_conf_.max_chassis_miss_num() * control_conf_.chassis_period())) {
    AERROR(
        "controller/controller_agent, ControllerAgent::CheckTimestamp",
        "chassis msg lost for " << std::setprecision(6) << chassis_diff << "s");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - local_view.trajectory().header().timestamp_sec();
  if (trajectory_diff > (control_conf_.max_planning_miss_num() *
                         control_conf_.trajectory_period())) {
    AERROR("controller/controller_agent, ControllerAgent::CheckTimestamp",
           "trajectory msg lost for " << std::setprecision(6) << trajectory_diff
                                      << "s");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "trajectory msg timeout");
  }
  return Status::OK();
}

}  // namespace control
}  // namespace autoagric