#include "common/test/static_trajectory_loader.h"

#include "common/macro.h"
#include "common/math/vec2d.h"
#include "control/common/pb3_ros_msgs.h"
#include "geometry_msgs/Vector3.h"
#include "hlbc/TrajectoryPoint.h"
#include "planning/reference_line/discrete_points_trajectory_smoother.h"
#include "std_msgs/ColorRGBA.h"

namespace autoagric {
namespace common {
namespace test {

using autoagric::common::math::Vec2d;
using autoagric::control::common::TrajectoryVisualizer;
using autoagric::planning::ADCTrajectory;
using autoagric::planning::AnchorPoint;
using autoagric::planning::DiscretePointsTrajectorySmoother;
using autoagric::planning::TrajectorySmootherConfig;

StaticTrajectoryLoader::StaticTrajectoryLoader(ros::NodeHandle& nh,
                                               std::string& file_path)
    : nh_(nh), static_trajectory_file_path_(file_path) {
  AINFO("Load static trajectory from " << file_path);
}

void StaticTrajectoryLoader::Init() {
  pose_reader_ = std::make_unique<ros::Subscriber>(nh_.subscribe(
      "/current_pose", 1, &StaticTrajectoryLoader::OnLocalization, this));

  local_trajectory_writer_ = std::make_unique<ros::Publisher>(
      nh_.advertise<hlbc::Trajectory>("static_trajectory", 1));

  global_trajectory_writer_ = std::make_unique<ros::Timer>(nh_.createTimer(
      ros::Duration(1), &StaticTrajectoryLoader::Visualize, this));

  loader_.reset(new TrajectoryLoader());

  loader_->Load<hlbc::Trajectory, hlbc::TrajectoryPoint>(
      static_trajectory_file_path_, &global_trajectory_);

  trajectory_length_ = global_trajectory_.trajectory_point.size();

  current_start_index_ = 0;
}

void StaticTrajectoryLoader::Visualize(const ros::TimerEvent& e) {
  visualizer_->Proc({markers_});
}

void StaticTrajectoryLoader::InitVisualizer(const std::string& name,
                                            const geometry_msgs::Vector3& scale,
                                            const std_msgs::ColorRGBA& color) {
  // global_trajectory_.header.stamp = ros::Time::now();
  global_trajectory_.header.frame_id = "/map";

  visualizer_.reset(new TrajectoryVisualizer(nh_, {name}));

  visualizer_->Init();

  markers_ =
      TrajectoryVisualizer::toMarkerArray(global_trajectory_, scale, color);
}

void StaticTrajectoryLoader::Smooth(const TrajectorySmootherConfig& config) {
  smoother_.reset(new DiscretePointsTrajectorySmoother(config));

  std::vector<AnchorPoint> anchor_points;

  ADCTrajectory trajectory;
  ADCTrajectory smoothed_trajectory;

  control::pb3::fromMsg(global_trajectory_, &trajectory);

  for (int i = 0; i < trajectory_length_; i++) {
    const auto& trajectory_point = trajectory.trajectory_point(i);
    AnchorPoint anchor_point;
    anchor_point.path_point.CopyFrom(trajectory_point.path_point());
    anchor_point.lateral_bound = config.max_lateral_boundary_bound();
    anchor_point.longitudinal_bound = config.longitudinal_boundary_bound();
    anchor_point.enforced = i == 0                        ? true
                            : i == trajectory_length_ - 1 ? true
                                                          : false;
    anchor_points.emplace_back(anchor_point);
  }

  smoother_->SetAnchorPoints(anchor_points);

  if (!smoother_->Smooth(trajectory, &smoothed_trajectory)) {
    AERROR("Unable to smoother trajectory");
    return;
  }

  AINFO(smoothed_trajectory.DebugString());

  global_trajectory_ = control::pb3::toMsg(smoothed_trajectory);
}

void StaticTrajectoryLoader::OnLocalization(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  current_start_index_ = QueryNearestPointByPoistion(
      msg->pose.position.x, msg->pose.position.y, current_start_index_ - 2);

  hlbc::Trajectory local_trajectory;

  double now_time =
      global_trajectory_.trajectory_point[current_start_index_].relative_time;

  for (int i = std::max<int>(current_start_index_ - 1, 0);
       i < std::min<int>(current_start_index_ + 20, trajectory_length_); i++) {
    auto& point = global_trajectory_.trajectory_point[i];
    point.relative_time -= now_time;
    local_trajectory.trajectory_point.push_back(point);
  }

  local_trajectory.header = msg->header;

  local_trajectory_writer_->publish(local_trajectory);
  // visualizer_->Proc({markers_});
}

int StaticTrajectoryLoader::QueryNearestPointByPoistion(const double x,
                                                        const double y,
                                                        int index) {
  if (index >= trajectory_length_) return trajectory_length_ - 1;
  if (index < 0) index = 0;

  double min_index = index;
  double min_dist = 1e19;

  Vec2d com(x, y);

  for (int i = index; i < trajectory_length_; i++) {
    Vec2d tmp(global_trajectory_.trajectory_point[i].path_point.x,
              global_trajectory_.trajectory_point[i].path_point.y);
    double dist_square = com.DistanceSquareTo(tmp);

    if (dist_square < min_dist) {
      min_dist = dist_square;
      min_index = i;
    }
  }

  return min_index;
}

}  // namespace test
}  // namespace common
}  // namespace autoagric