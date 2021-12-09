#include "control/controller/controller_agent.h"

#include <ros/ros.h>

#include <memory>
#include <utility>

#include "autoagric/common/error_code.pb.h"
#include "common/macro.h"
#include "control/common/control_gflags.h"
#include "control/controller/lat_controller.h"
#include "control/controller/mpc_controller.h"

namespace autoagric {
namespace control {

using autoagric::common::ErrorCode;
using autoagric::common::Status;

// void ControllerAgent::RegisterControllers(const ControlConf *control_conf) {
//   AINFO << "Only support MPC controller or Lat + Lon controllers as of now";
//   for (auto active_controller : control_conf->active_controllers()) {
//     switch (active_controller) {
//       case ControlConf::MPC_CONTROLLER:
//         controller_factory_.Register(
//             ControlConf::MPC_CONTROLLER,
//             []() -> Controller * { return new MPCController(); });
//         break;
//       case ControlConf::LAT_CONTROLLER:
//         controller_factory_.Register(
//             ControlConf::LAT_CONTROLLER,
//             []() -> Controller * { return new LatController(); });
//         break;
//       case ControlConf::LON_CONTROLLER:
//         controller_factory_.Register(
//             ControlConf::LON_CONTROLLER,
//             []() -> Controller * { return new LonController(); });
//         break;
//       default:
//         AERROR << "Unknown active controller type:" << active_controller;
//     }
//   }
// }

// Status ControllerAgent::InitializeConf(const ControlConf *control_conf) {
//   if (!control_conf) {
//     AERROR << "control_conf is null";
//     return Status(ErrorCode::CONTROL_INIT_ERROR, "Failed to load config");
//   }
//   control_conf_ = control_conf;
//   for (auto controller_type : control_conf_->active_controllers()) {
//     auto controller = controller_factory_.CreateObject(
//         static_cast<ControlConf::ControllerType>(controller_type));
//     if (controller) {
//       controller_list_.emplace_back(std::move(controller));
//     } else {
//       AERROR << "Controller: " << controller_type << "is not supported";
//       return Status(ErrorCode::CONTROL_INIT_ERROR,
//                     "Invalid controller type:" + controller_type);
//     }
//   }
//   return Status::OK();
// }

Status ControllerAgent::InitializeConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR("control_conf is null");
    return Status(ErrorCode::CONTROL_INIT_ERROR, "Failed to load config");
  }
  control_conf_ = control_conf;
  return Status::OK();
}

Status ControllerAgent::Init(std::shared_ptr<DependencyInjector> injector,
                             const ControlConf *control_conf) {
  injector_ = injector;
  // std::make_shared<DependencyInjector>();

  AINFO("Control init, starting ...");

  AERROR_IF(!InitializeConf(control_conf).ok(), "Failed to initialize config.");

  /**
   * @todo(chufan) add controller factory
   */
  private_controller_ = std::make_shared<MPCController>();
  controller_ = private_controller_;

  if (!FLAGS_use_control_submodules &&
      !controller_->Init(injector_, control_conf_).ok()) {
    ADEBUG("original control");
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "fail to init controller: " + controller_->Name());
  }
  return Status::OK();
}

Status ControllerAgent::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
    control::ControlCommand *cmd) {
  ADEBUG("controller:" << controller_->Name() << " processing ...");
  double start_timestamp = ros::Time::now().toNSec();
  auto status = controller_->ComputeControlCommand(localization, chassis,
                                                   trajectory, cmd);
  double end_timestamp = ros::Time::now().toNSec();
  const double time_diff_ms = (end_timestamp - start_timestamp) / 1e6;

  ADEBUG("controller: " << controller_->Name()
                        << " calculation time is: " << time_diff_ms << " ms.");
  cmd->mutable_latency_stats()->add_controller_time_ms(time_diff_ms);

  return status;
}

Status ControllerAgent::Reset() {
  ADEBUG("controller:" << controller_->Name() << " reset...");
  controller_->Reset();

  return Status::OK();
}

}  // namespace control
}  // namespace autoagric
