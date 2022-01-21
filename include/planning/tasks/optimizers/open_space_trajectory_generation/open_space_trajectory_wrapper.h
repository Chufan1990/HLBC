#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/planning/open_space_task_config.pb.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "common/math/vec2d.h"
#include "common/util/trajectory_visualizer.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_generator.h"

namespace autoagric {
namespace planning {

struct OpenSpaceTrajectoryThreadData {
  std::vector<common::TrajectoryPoint> stitching_trajectory;
  std::vector<double> cur_pose;
  std::vector<double> end_pose;
  std::vector<double> XYbounds;
  double rotate_angle;
  common::math::Vec2d translate_origin;
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec;
};

class OpenSpaceTrajectoryWrapper {
 public:
  OpenSpaceTrajectoryWrapper() = default;

  OpenSpaceTrajectoryWrapper(ros::NodeHandle& nh);

  bool Init();

  bool Proc();

 private:
  void OnLocalization(const geometry_msgs::PoseStampedConstPtr& msg);

  void OnObstacle(const autoware_msgs::DetectedObjectArrayConstPtr& msg);

  void OnDestination(const geometry_msgs::PoseStampedConstPtr& msg);

  void GetRoiBoundary(const double sx, const double sy, const double sphi,
                      const double ex, const double ey, const double ephi,
                      const double vx, const double vy, const double vphi,
                      std::vector<double>* XYbounds);

  bool GetVirtualParkingLot(
      const double sx, const double sy, const double stheta, const double vx,
      const double vy, const double vtheta,
      std::vector<std::vector<common::math::Vec2d>>* obstacles_vertices_vec,
      std::vector<double>* end_pose);

  std::vector<common::math::Vec2d> CenterToRectangle(
      const double cx, const double cy, const double cphi, const double left,
      const double right, const double front, const double rear);

  void Visualize(const ros::TimerEvent& e);

  ros::NodeHandle nh_;

  std::unique_ptr<ros::AsyncSpinner> spinner_;

  std::unique_ptr<ros::Subscriber> localization_reader_;

  std::unique_ptr<ros::Subscriber> stitching_trajectory_reader_;

  std::unique_ptr<ros::Subscriber> obstacle_reader_;

  std::unique_ptr<ros::Subscriber> destination_reader_;

  std::unique_ptr<ros::Publisher> trajectory_writer_;

  DiscretizedTrajectory optimized_trajectory_;

  std::timed_mutex localization_copy_done_;

  std::timed_mutex chassis_copy_done_;

  std::timed_mutex obstacles_copy_done_;

  std::timed_mutex destination_copy_done_;

  std::shared_ptr<DependencyInjector> injector_;

  std::unique_ptr<OpenSpaceTrajectoryGenerator>
      open_space_trajectory_generator_;

  OpenSpaceTrajectoryThreadData thread_data_;

  std::atomic<bool> localization_ready_{false};

  std::atomic<bool> obstacles_ready_{false};

  std::atomic<bool> destination_ready_{false};

  std::atomic<bool> data_ready_{false};

  std::atomic<bool> trajectory_updated_{false};

  std::unique_ptr<common::util::TrajectoryVisualizer> warm_start_visualizer_;

  std::unique_ptr<common::util::TrajectoryVisualizer>
      optimized_trajectory_visualizer_;

  std::unique_ptr<common::util::TrajectoryVisualizer> obstacle_visualizer_;

  std::unordered_map<std::string, visualization_msgs::MarkerArray>
      warm_start_markers_;

  std::unordered_map<std::string, visualization_msgs::MarkerArray>
      optimized_trajectory_markers_;

  std::unordered_map<std::string, visualization_msgs::MarkerArray>
      obstacle_markers_;

  std::unique_ptr<ros::Timer> trajectory_marker_writer_;

  OpenSpaceTrajectoryTaskConfig config_;
};

}  // namespace planning
}  // namespace autoagric