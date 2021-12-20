#pragma once

#include <memory>
#include <string>
#include <utility>

#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/planning/planning.pb.h"
#include "autoagric/planning/reference_line_smoother_config.pb.h"
#include "common/test/trajectory_loader.h"
#include "control/common/trajectory_visualizer.h"
#include "geometry_msgs/PoseStamped.h"
#include "hlbc/Trajectory.h"
#include "planning/reference_line/discrete_points_trajectory_smoother.h"
#include "ros/ros.h"

namespace autoagric {
namespace common {
namespace test {

class StaticTrajectoryLoader {
 public:
  StaticTrajectoryLoader() = default;

  StaticTrajectoryLoader(ros::NodeHandle& nh, std::string& file_path);

  void Init();

  void Smooth(const planning::TrajectorySmootherConfig& config);

  void InitVisualizer(const std::string& name,
                      const geometry_msgs::Vector3& scale,
                      const std_msgs::ColorRGBA& color);

 private:
  void Visualize(const ros::TimerEvent& e);

  void OnLocalization(const geometry_msgs::PoseStampedConstPtr& msg);

  std::pair<int, double> QueryNearestPointByPoistion(const double x,
                                                     const double y, int index);

  int current_start_index_;

  int trajectory_length_;

  ros::NodeHandle nh_;

  std::unique_ptr<ros::Subscriber> pose_reader_;

  std::unique_ptr<ros::Publisher> local_trajectory_writer_;

  std::unique_ptr<ros::Timer> global_trajectory_writer_;

  std::unique_ptr<TrajectoryLoader> loader_;

  std::string static_trajectory_file_path_;

  hlbc::Trajectory global_trajectory_;

  planning::TrajectorySmootherConfig config_;

  std::unique_ptr<planning::TrajectorySmoother> smoother_;

  std::unique_ptr<control::TrajectoryVisualizer> visualizer_;

  control::TrajectoryVisualizer::MarkerType markers_;
};
}  // namespace test
}  // namespace common
}  // namespace autoagric