#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "autoagric/canbus/chassis.pb.h"
#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/localization/localization.pb.h"
#include "autoagric/planning/planning.pb.h"
#include "autoagric/planning/reference_line_smoother_config.pb.h"
#include "common/test/trajectory_loader.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "control/common/pb3_ros_msgs.h"
#include "control/common/trajectory_visualizer.h"
#include "hlbc/Trajectory.h"
#include "planning/common/ego_info.h"
#include "planning/reference_line/discrete_points_trajectory_smoother.h"

namespace autoagric {
namespace common {
namespace test {

class StaticTrajectoryLoader {
 public:
  StaticTrajectoryLoader() = default;

  StaticTrajectoryLoader(ros::NodeHandle& nh, std::string& file_path);

  void Init();

  void Proc();

  void Smooth(const planning::TrajectorySmootherConfig& config);

  void InitVisualizer(const std::string& name,
                      const geometry_msgs::Vector3& scale,
                      const std_msgs::ColorRGBA& color);

 private:
  void Visualize(const ros::TimerEvent& e);

  void OnLocalization(const geometry_msgs::PoseStampedConstPtr& msg1,
                      const geometry_msgs::TwistStampedConstPtr& msg2);

  void OnChassis(const geometry_msgs::TwistStampedConstPtr& msg);

  hlbc::Trajectory GenerateLocalProfile(const int begin, const int end,
                                        const double now) const;

  std::pair<int, double> QueryNearestPointByPoistion(const double x,
                                                     const double y, int index);

  int current_start_index_;

  int trajectory_length_;

  ros::NodeHandle nh_;

  std::unique_ptr<ros::AsyncSpinner> spinner_;

  // std::unique_ptr<ros::Subscriber> localization_reader_;

  typedef message_filters::sync_policies::ApproximateTime<
      geometry_msgs::PoseStamped, geometry_msgs::TwistStamped>
      ApproximateSyncPolicy;

  std::unique_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>>
      localization_reader_;

  std::unique_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>>
      loc_message_filter_;

  std::unique_ptr<message_filters::Subscriber<geometry_msgs::TwistStamped>>
      imu_message_filter_;

  std::unique_ptr<ros::Subscriber> chassis_reader_;

  std::unique_ptr<ros::Publisher> local_trajectory_writer_;

  std::unique_ptr<ros::Timer> global_trajectory_writer_;

  std::unique_ptr<TrajectoryLoader> loader_;

  std::string static_trajectory_file_path_;

  hlbc::Trajectory global_trajectory_;

  planning::TrajectorySmootherConfig config_;

  std::unique_ptr<planning::TrajectorySmoother> smoother_;

  std::unique_ptr<control::TrajectoryVisualizer> visualizer_;

  control::TrajectoryVisualizer::MarkerType markers_;

  planning::EgoInfo ego_info_;

  std::unique_ptr<common::VehicleStateProvider> vehicle_state_provider_;

  localization::LocalizationEstimate latest_localization_;

  canbus::Chassis latest_chassis_;

  std::timed_mutex localization_copy_done_;

  std::timed_mutex chassis_copy_done_;
};
}  // namespace test
}  // namespace common
}  // namespace autoagric