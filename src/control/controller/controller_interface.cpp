#include "control/controller/controller_interface.h"

#include "autoagric/common/error_code.pb.h"
#include "common/util/file.h"
#include "control/common/control_gflags.h"
#include "planning/common/planning_gflags.h"
#include "planning/reference_line/discrete_points_trajectory_smoother.h"

namespace autoagric {
namespace control {

using autoagric::common::ErrorCode;
using autoagric::common::Status;

bool ControllerInterface::Init() {
  injector_ = std::make_shared<DependencyInjector>();

  AERROR_IF(
      !autoagric::common::util::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_),
      "Unable to load control conf file: " << FLAGS_control_conf_file);

  AERROR_IF(!autoagric::common::util::GetProtoFromFile(
                FLAGS_discrete_points_smoother_config_filename,
                &trajectory_smoother_conf_),
            "Unable to load control conf file: "
                << FLAGS_discrete_points_smoother_config_filename);

  AINFO("Conf file: " << FLAGS_control_conf_file << " is loaded.");

  ADEBUG("FLAGS_use_control_submodules: " << FLAGS_use_control_submodules);

  smoother_ = std::unique_ptr<planning::TrajectorySmoother>(
      new planning::DiscretePointsTrajectorySmoother(
          trajectory_smoother_conf_));

  if (!controller_agent_.Init(injector_, &control_conf_).ok()) return false;
  return true;
}

Status ControllerInterface::CheckInput(LocalView* local_view) {
  if (local_view->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100, "planning has no trajectory point. ");

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

Status ControllerInterface::CheckTimestamp(const LocalView& local_view) {
  if (!control_conf_.enable_input_timestamp_check() ||
      control_conf_.is_control_test_mode()) {
    ADEBUG("Skip input timestamp check by gflags");
    return Status::OK();
  }

  double current_timestamp = ros::Time::now().toSec();

  double localization_diff =
      current_timestamp - local_view.localization().header().timestamp_sec();
  if (localization_diff > (control_conf_.max_localization_miss_num() *
                           control_conf_.localization_period())) {
    AERROR("localization msg lost for " << std::setprecision(6)
                                        << localization_diff << "s");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "localization msg timeout");
  }

  double chassis_diff =
      current_timestamp - local_view.chassis().header().timestamp_sec();
  if (chassis_diff >
      (control_conf_.max_chassis_miss_num() * control_conf_.chassis_period())) {
    AERROR("chassis msg lost for " << std::setprecision(6) << chassis_diff
                                   << "s");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - local_view.trajectory().header().timestamp_sec();
  if (trajectory_diff > (control_conf_.max_planning_miss_num() *
                         control_conf_.trajectory_period())) {
    AERROR("trajectory msg lost for " << std::setprecision(6) << trajectory_diff
                                      << "s");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "trajectory msg timeout");
  }
  return Status::OK();
}

Status ControllerInterface::ComputeControlCommand(
    const localization::LocalizationEstimate* localization,
    const canbus::Chassis* chassis, const planning::ADCTrajectory* trajectory,
    control::ControlCommand* control_command) {
  return controller_agent_.ComputeControlCommand(localization, chassis,
                                                 trajectory, control_command);
}

}  // namespace control
}  // namespace autoagric
