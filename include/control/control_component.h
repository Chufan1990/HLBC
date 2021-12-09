#pragma once

#include <memory>
#include <string>

#include "autoagric/control/local_view.pb.h"
#include "autoagric/planning/reference_line_smoother_config.pb.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/Lane.h"
#include "control/common/trajectory_visualizer.h"
#include "control/controller/controller_agent.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "planning/reference_line/trajectory_smoother.h"
#include "ros/ros.h"

namespace autoagric {
namespace control {

class ControlComponent final {
 public:
  ControlComponent(ros::NodeHandle& nh);

  ControlComponent() = default;

  bool Init();

  bool Proc();

 private:
  void OnChassis(const geometry_msgs::TwistStampedConstPtr& msg);

  void OnPlanning(const autoware_msgs::LaneConstPtr& msg);

  void Onlocalization(const geometry_msgs::PoseStampedConstPtr& msg);

  autoagric::common::Status ProduceControlCommand(
      ControlCommand* control_command);

 private:
  autoagric::common::Status CheckInput(LocalView* local_view);

  autoagric::common::Status CheckTimestamp(const LocalView& local_view);

  ros::NodeHandle nh_;

  ros::NodeHandle visualizer_nh_;

  std::shared_ptr<ros::Subscriber> chassis_reader_;

  std::shared_ptr<ros::Subscriber> localization_reader_;

  std::shared_ptr<ros::Subscriber> planning_reader_;

  std::shared_ptr<ros::Publisher> control_cmd_writer_;

  LocalView local_view_;

  ControlCommand control_command_;

  ControllerAgent controller_agent_;

  std::shared_ptr<DependencyInjector> injector_;

  std::unique_ptr<common::TrajectoryVisualizer> visualizer_;

  ControlConf control_conf_;

  bool estop_;

  std::string estop_reason_;

  std::unique_ptr<ros::AsyncSpinner> spinner_;

  std::unique_ptr<planning::TrajectorySmoother> smoother_;

  planning::TrajectorySmootherConfig trajectory_smoother_conf_;
};

}  // namespace control
}  // namespace autoagric