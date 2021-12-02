#pragma once

#include "autoagric/control/control_command.pb.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/Lane.h"
#include "common/status/status.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"

namespace autoagric {
namespace control {

class ControlComponent final {
 public:
  ControlComponent(ros::NodeHandle nh);

  bool Init();
  bool Proc();

 private:
  void OnChassis();

  void OnPlanning();

  void Onlocalization();

  common::Status ProduceControlCommand(ControlCommand* control_command);

 private:
  LocalView local_view_;
  ControllerInterface controller_interface_;
  ros::Nodehandle nh_;
}

}  // namespace control
}  // namespace autoagric