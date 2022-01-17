#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/planning/open_space_trajectory_generator_config.pb.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "common/math/vec2d.h"
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
                      std::vector<double>* XYbounds);

  std::vector<common::math::Vec2d> CenterToRectangle(
      const double cx, const double cy, const double cphi, const double left,
      const double right, const double front, const double rear);

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

  OpenSpaceTrajectoryGeneratorConfig config_;

  std::atomic<bool> localization_ready_;

  std::atomic<bool> obstacles_ready_;

  std::atomic<bool> destination_ready_;

  std::atomic<bool> data_ready_;

  std::atomic<bool> trajectory_updated_;
};

}  // namespace planning
}  // namespace autoagric