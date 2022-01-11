#include "planning/static/static_path_wrapper.h"

#include <geometry_msgs/Vector3.h>
#include <message_filters/subscriber.h>
#include <std_msgs/ColorRGBA.h>

#include <chrono>
#include <memory>

#include "common/macro.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "control/common/pb3_ros_msgs.h"
#include "hlbc/TrajectoryPoint.h"
#include "planning/common/speed_profile_generator.h"
#include "planning/reference_line/discrete_points_trajectory_smoother.h"

namespace autoagric {
namespace planning {

namespace {
constexpr double kDoubleEpsilon = 1e-3;
}

using autoagric::control::TrajectoryVisualizer;
using common::util::StaticPathResult;
using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;

StaticPathWrapper::StaticPathWrapper(ros::NodeHandle& nh,
                                     std::string& file_path)
    : nh_(nh), static_trajectory_file_path_(file_path) {
  AINFO("Load static trajectory from " << file_path);
}

bool StaticPathWrapper::Init(const StaticPathConfig& config) {
  spinner_ = std::make_unique<ros::AsyncSpinner>(0);

  chassis_reader_ = std::make_unique<ros::Subscriber>(nh_.subscribe(
      "/vehicle/feedback", 1, &StaticPathWrapper::OnChassis, this));

  loc_message_filter_.reset(
      new message_filters::Subscriber<geometry_msgs::PoseStamped>(
          nh_, "/current_pose", 10));
  imu_message_filter_.reset(
      new message_filters::Subscriber<geometry_msgs::TwistStamped>(
          nh_, "/vehicle/twist", 10));

  localization_reader_.reset(new Synchronizer<ApproximateSyncPolicy>(
      ApproximateSyncPolicy(10), *loc_message_filter_, *imu_message_filter_));

  localization_reader_->registerCallback(
      boost::bind(&StaticPathWrapper::OnLocalization, this, _1, _2));

  // localization_reader_ = std::make_unique<ros::Subscriber>(nh_.subscribe(
  //     "/current_pose", 1, &StaticPathWrapper::OnLocalization, this));

  local_trajectory_writer_ = std::make_unique<ros::Publisher>(
      nh_.advertise<hlbc::Trajectory>("static_trajectory", 1));

  global_trajectory_writer_ = std::make_unique<ros::Timer>(
      nh_.createTimer(ros::Duration(1), &StaticPathWrapper::Visualize, this));

  path_generator_.reset(new StaticPathGenerator(static_trajectory_file_path_));

  vehicle_state_provider_.reset(new common::VehicleStateProvider());

  if (!path_generator_->Init(config)) {
    AERROR("StaticPathGenerator Init failed");
    return false;
  }

  if (!path_generator_->Proc()) {
    AERROR("Generating static path failed");
    return false;
  }
  return true;
}

void StaticPathWrapper::Visualize(const ros::TimerEvent& e) {
  visualizer_->Proc({markers_});
}

void StaticPathWrapper::InitVisualizer(const std::string& name,
                                       const geometry_msgs::Vector3& scale,
                                       const std_msgs::ColorRGBA& color) {
  // global_trajectory_.header.stamp = ros::Time::now();
  std_msgs::Header header;
  header.frame_id = "/map";
  header.stamp = ros::Time::now();

  visualizer_.reset(new TrajectoryVisualizer(nh_, {name}));

  visualizer_->Init();

  markers_ = TrajectoryVisualizer::toMarkerArray(
      path_generator_->Path().x, path_generator_->Path().y,
      path_generator_->Path().phi, header, scale, color);
}

bool StaticPathWrapper::Proc() {
  spinner_->start();
  ros::Rate loop_rate(10);
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

    auto result = std::move(path_generator_->GenerateLocalProfile(
        vehicle_state_provider_->x(), vehicle_state_provider_->y()));

    for (size_t i = 0; i < result.x.size(); i++) {
      ADEBUG(std::setprecision(3)
             << std::fixed << "x: " << result.x[i] << " y: " << result.y[i]
             << " theta: " << result.phi[i] << " v: " << result.v[i]
             << " a: " << result.a[i] << " s: " << result.accumulated_s[i]
             << " t: " << result.relative_time[i]);
    }

    local_trajectory_writer_->publish(GenerateLocalProfile(result));

    visualizer_->Proc({markers_});
    loop_rate.sleep();
  }
  spinner_->stop();
}

hlbc::Trajectory StaticPathWrapper::GenerateLocalProfile(
    const StaticPathResult& result) const {
  hlbc::Trajectory trajectory;
  for (size_t i = 0; i < result.x.size(); i++) {
    hlbc::TrajectoryPoint point;
    point.path_point.x = result.x[i];
    point.path_point.y = result.y[i];
    point.path_point.theta = result.phi[i];
    point.path_point.kappa = result.kappa[i];
    point.path_point.s = result.accumulated_s[i];
    point.a = result.a[i];
    point.v = result.v[i];
    point.relative_time = result.relative_time[i];
    trajectory.trajectory_point.push_back(point);
  }
  trajectory.header.stamp = ros::Time::now();
  trajectory.header.frame_id = "map";
  return trajectory;
}

void StaticPathWrapper::OnChassis(
    const geometry_msgs::TwistStampedConstPtr& msg) {
  std::unique_lock<std::timed_mutex> locker(chassis_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    control::pb3::fromMsg(msg, &latest_chassis_);
  }
}

void StaticPathWrapper::OnLocalization(
    const geometry_msgs::PoseStamped::ConstPtr& msg1,
    const geometry_msgs::TwistStamped::ConstPtr& msg2) {
  std::unique_lock<std::timed_mutex> locker(localization_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    control::pb3::fromMsg(msg1, &latest_localization_);
    control::pb3::fromMsg(msg2, &latest_localization_);
  }
}

}  // namespace planning
}  // namespace autoagric