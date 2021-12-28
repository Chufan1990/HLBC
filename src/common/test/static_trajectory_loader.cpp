#include "common/test/static_trajectory_loader.h"

#include <geometry_msgs/Vector3.h>
#include <message_filters/subscriber.h>
#include <std_msgs/ColorRGBA.h>

#include <chrono>
#include <memory>

#include "common/macro.h"
#include "common/math/vec2d.h"
#include "control/common/pb3_ros_msgs.h"
#include "hlbc/TrajectoryPoint.h"
#include "planning/common/speed_profile_generator.h"
#include "planning/reference_line/discrete_points_trajectory_smoother.h"

namespace autoagric {
namespace common {
namespace test {

using autoagric::control::TrajectoryVisualizer;
using autoagric::planning::ADCTrajectory;
using autoagric::planning::AnchorPoint;
using autoagric::planning::DiscretePointsTrajectorySmoother;
using autoagric::planning::TrajectorySmootherConfig;
using common::math::Vec2d;
using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;

StaticTrajectoryLoader::StaticTrajectoryLoader(ros::NodeHandle& nh,
                                               std::string& file_path)
    : nh_(nh), static_trajectory_file_path_(file_path) {
  AINFO("Load static trajectory from " << file_path);
}

void StaticTrajectoryLoader::Init() {
  spinner_ = std::make_unique<ros::AsyncSpinner>(0);

  chassis_reader_ = std::make_unique<ros::Subscriber>(nh_.subscribe(
      "/vehicle/feedback", 1, &StaticTrajectoryLoader::OnChassis, this));

  loc_message_filter_.reset(
      new message_filters::Subscriber<geometry_msgs::PoseStamped>(
          nh_, "/current_pose", 10));
  imu_message_filter_.reset(
      new message_filters::Subscriber<geometry_msgs::TwistStamped>(
          nh_, "/vehicle/twist", 10));

  localization_reader_.reset(new Synchronizer<ApproximateSyncPolicy>(
      ApproximateSyncPolicy(10), *loc_message_filter_, *imu_message_filter_));

  localization_reader_->registerCallback(
      boost::bind(&StaticTrajectoryLoader::OnLocalization, this, _1, _2));

  // localization_reader_ = std::make_unique<ros::Subscriber>(nh_.subscribe(
  //     "/current_pose", 1, &StaticTrajectoryLoader::OnLocalization, this));

  local_trajectory_writer_ = std::make_unique<ros::Publisher>(
      nh_.advertise<hlbc::Trajectory>("static_trajectory", 1));

  global_trajectory_writer_ = std::make_unique<ros::Timer>(nh_.createTimer(
      ros::Duration(1), &StaticTrajectoryLoader::Visualize, this));

  vehicle_state_provider_.reset(new common::VehicleStateProvider());

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

void StaticTrajectoryLoader::Proc() {
  spinner_->start();
  ros::Rate loop_rate(10);
  // AINFO("latest_localization_.IsInitialized(): "
  //       << latest_localization_.has_header());
  // AINFO("latest_chassis_.IsInitialized(): " << latest_chassis_.has_header());
  while (ros::ok() && (!latest_localization_.has_header() ||
                       !latest_chassis_.has_header())) {
    AINFO_EVERY(1000, "Waiting for localization and chassis message ...");
    AWARN_IF(!latest_localization_.has_header(),
             "Localization message not received.");
    AWARN_IF(!latest_chassis_.has_header(), "Chassis message not received.");
    AWARN_EVERY(1000, latest_localization_.DebugString());
    AWARN_EVERY(1000, latest_chassis_.DebugString());
    loop_rate.sleep();
  }

  while (ros::ok()) {
    {
      std::lock(localization_copy_done_, chassis_copy_done_);
      // make sure both already-locked mutexes are unlocked at the end of
      // scope
      std::lock_guard<std::timed_mutex> lock2(localization_copy_done_,
                                              std::adopt_lock);
      std::lock_guard<std::timed_mutex> lock3(chassis_copy_done_,
                                              std::adopt_lock);
      vehicle_state_provider_->Update(latest_localization_, latest_chassis_);
    }

    auto matched_index_and_distance = QueryNearestPointByPoistion(
        vehicle_state_provider_->x(), vehicle_state_provider_->y(),
        current_start_index_ - 1);

    current_start_index_ =
        (matched_index_and_distance.second > 0.6) &&
                (matched_index_and_distance.first == current_start_index_ - 1)
            ? 0
            : matched_index_and_distance.first;

    size_t starting_index = std::max<int>(current_start_index_ - 1, 0);

    size_t num_of_knots =
        std::min<size_t>(20, trajectory_length_ - starting_index);

    auto matched_point =
        global_trajectory_.trajectory_point[current_start_index_];
    // auto start_point = global_trajectory_.trajectory_point[starting_index];
    // auto end_point =
    //     global_trajectory_.trajectory_point[starting_index + num_of_knots];

    // /// based on if vehicle feedback gives negative speed value when vehicle
    // /// moving backward. Calculation requires semi-positive speed value.
    // start_point.v = std::fabs(start_point.v);
    // end_point.v = std::fabs(end_point.v);
    // matched_point.v = std::fabs(matched_point.v);

    const double time_now = matched_point.relative_time;

    // const double trajectory_length =
    //     end_point.path_point.s - start_point.path_point.s;

    // common::TrajectoryPoint matched_point_pb;
    // control::pb3::fromMsg(matched_point, &matched_point_pb);
    // ego_info_.Update(matched_point_pb,
    //                  vehicle_state_provider_->vehicle_state());

    // planning::SpeedData speed_data =
    //     std::move(std::fabs(end_point.v) < 1e-3
    //                   ?
    //                   planning::SpeedProfileGenerator::GenerateFallbackSpeed(
    //                         &ego_info_, trajectory_length)
    //                   : planning::SpeedProfileGenerator::
    //                         GenerateFixedDistanceCreepProfile(
    //                             trajectory_length, std::fabs(end_point.v)));

    local_trajectory_writer_->publish(std::move(
        GenerateLocalProfile(starting_index, starting_index + num_of_knots, time_now)));

    visualizer_->Proc({markers_});
    loop_rate.sleep();
  }
  spinner_->stop();
}

hlbc::Trajectory StaticTrajectoryLoader::GenerateLocalProfile(
    const int begin, const int end, const double now) const {
  hlbc::Trajectory trajectory;
  for (size_t i = begin; i < end; i++) {
    auto point = global_trajectory_.trajectory_point[i];
    point.relative_time -= now;
    trajectory.trajectory_point.push_back(point);
  }
  trajectory.header.stamp = ros::Time::now();
  trajectory.header.frame_id = "map";
  return trajectory;
}

void StaticTrajectoryLoader::OnChassis(
    const geometry_msgs::TwistStampedConstPtr& msg) {
  std::unique_lock<std::timed_mutex> locker(chassis_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    control::pb3::fromMsg(msg, &latest_chassis_);
  }
}

void StaticTrajectoryLoader::OnLocalization(
    const geometry_msgs::PoseStamped::ConstPtr& msg1,
    const geometry_msgs::TwistStamped::ConstPtr& msg2) {
  std::unique_lock<std::timed_mutex> locker(localization_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    control::pb3::fromMsg(msg1, &latest_localization_);
    control::pb3::fromMsg(msg2, &latest_localization_);
  }
}

std::pair<int, double> StaticTrajectoryLoader::QueryNearestPointByPoistion(
    const double x, const double y, int index) {
  if (index >= trajectory_length_)
    return std::make_pair(trajectory_length_ - 1, 1e9);
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

  return std::make_pair(min_index, min_dist);
}

}  // namespace test
}  // namespace common
}  // namespace autoagric