#include "planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_wrapper.h"

#include <boost/bind.hpp>
#include <string>
#include <unordered_map>

#include "common/configs/vehicle_config_helper.h"
#include "common/macro.h"
#include "common/math/box2d.h"
#include "common/math/quaternion.h"
#include "common/status/status.h"
#include "common/util/file.h"
#include "hlbc/Trajectory.h"
#include "planning/open_space/coarse_trajectory_generator/node3d.h"

namespace autoagric {
namespace planning {

using common::Status;
using common::math::Box2d;
using common::math::Vec2d;

OpenSpaceTrajectoryWrapper::OpenSpaceTrajectoryWrapper(ros::NodeHandle& nh)
    : nh_(nh) {
  AINFO("....");
}

bool OpenSpaceTrajectoryWrapper::Init() {
  std::string open_space_trajectory_conf_file;
  if (!nh_.getParam("config", open_space_trajectory_conf_file)) {
    AERROR("Unable to retrived open space trajectory configuration file");
    return false;
  }

  std::string filename = absl::StrCat(std::string(std::getenv("HOME")),
                                      open_space_trajectory_conf_file);

  if (!common::util::GetProtoFromFile(filename, &config_)) {
    AERROR("Unable to load control conf file: "
           << open_space_trajectory_conf_file);
    return false;
  } else {
    AINFO("Configuration file " << open_space_trajectory_conf_file
                                << " is loaded: " << config_.DebugString());
  }

  std::string obstacle_message;
  if (!nh_.getParam("obstacle_message", obstacle_message)) {
    AERROR("Unable to retrived obstacle message topic name");
    return false;
  }

  std::string localization_message;
  if (!nh_.getParam("localization_message", localization_message)) {
    AERROR("Unable to retrived localization message topic name");
    return false;
  }

  std::string destination_message;
  if (!nh_.getParam("destination_message", destination_message)) {
    AERROR("Unable to retrived destination message topic name");
    return false;
  }

  std::string trajectory_message;
  if (!nh_.getParam("trajectory_message", trajectory_message)) {
    AERROR("Unable to retrived trajectory message topic name");
    return false;
  }

  spinner_ = std::make_unique<ros::AsyncSpinner>(0);

  obstacle_reader_ = std::make_unique<ros::Subscriber>(nh_.subscribe(
      obstacle_message, 1, &OpenSpaceTrajectoryWrapper::OnObstacle, this));

  destination_reader_ = std::make_unique<ros::Subscriber>(
      nh_.subscribe(destination_message, 1,
                    &OpenSpaceTrajectoryWrapper::OnDestination, this));

  localization_reader_ = std::make_unique<ros::Subscriber>(
      nh_.subscribe(localization_message, 1,
                    &OpenSpaceTrajectoryWrapper::OnLocalization, this));

  trajectory_writer_ = std::make_unique<ros::Publisher>(
      nh_.advertise<hlbc::Trajectory>(trajectory_message, 1));

  open_space_trajectory_generator_.reset(
      new OpenSpaceTrajectoryGenerator(config_));

  warm_start_visualizer_.reset(new common::util::TrajectoryVisualizer(nh_));

  optimized_trajectory_visualizer_.reset(
      new common::util::TrajectoryVisualizer(nh_));

  std::unordered_map<std::string,
                     std::pair<std_msgs::ColorRGBA, geometry_msgs::Vector3>>
      markers_properties;

  std_msgs::ColorRGBA color;
  geometry_msgs::Vector3 scale;

  color.a = 1.0;
  color.r = 0.5;
  color.g = 0.5;
  color.b = 0.1;
  scale.x = 0.2;
  scale.y = 0.2;
  scale.z = 0.2;
  markers_properties["arrows"] = std::make_pair(color, scale);
  markers_properties["points_and_lines"] = std::make_pair(color, scale);

  warm_start_visualizer_->Setup("warm_start", "/map", markers_properties);

  color.a = 1.0;
  color.r = 0.1;
  color.g = 1.0;
  color.b = 0.1;
  scale.x = 0.2;
  scale.y = 0.2;
  scale.z = 0.2;
  markers_properties["arrows"] = std::make_pair(color, scale);
  markers_properties["points_and_lines"] = std::make_pair(color, scale);

  optimized_trajectory_visualizer_->Setup("optimized", "/map",
                                          markers_properties);

  trajectory_marker_writer_ = std::make_unique<ros::Timer>(nh_.createTimer(
      ros::Duration(1), &OpenSpaceTrajectoryWrapper::Visualize, this));

  return true;
}

void OpenSpaceTrajectoryWrapper::Visualize(const ros::TimerEvent& e) {
  if (trajectory_updated_.load()) {
    optimized_trajectory_visualizer_->Publish(optimized_trajectory_markers_);
    warm_start_visualizer_->Publish(warm_start_markers_);
  }
}

void OpenSpaceTrajectoryWrapper::OnLocalization(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::timed_mutex> locker(localization_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    thread_data_.cur_pose.clear();
    thread_data_.cur_pose.push_back(msg->pose.position.x);
    thread_data_.cur_pose.push_back(msg->pose.position.y);
    thread_data_.cur_pose.push_back(common::math::QuaternionToHeading(
        msg->pose.orientation.w, msg->pose.orientation.x,
        msg->pose.orientation.y, msg->pose.orientation.z));
    localization_ready_.store(true);
  }
}

void OpenSpaceTrajectoryWrapper::OnObstacle(
    const autoware_msgs::DetectedObjectArrayConstPtr& msg) {
  std::unique_lock<std::timed_mutex> locker(obstacles_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    thread_data_.obstacles_vertices_vec.clear();
    for (const auto& object : msg->objects) {
      std::vector<Vec2d> obstacle_vertices;
      for (const auto& point : object.convex_hull.polygon.points) {
        obstacle_vertices.emplace_back(point.x, point.y);
      }
      thread_data_.obstacles_vertices_vec.emplace_back(
          std::move(obstacle_vertices));
      obstacles_ready_.store(true);
    }
  }
}

void OpenSpaceTrajectoryWrapper::OnDestination(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::timed_mutex> locker(destination_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    thread_data_.end_pose.clear();
    thread_data_.end_pose.push_back(msg->pose.position.x);
    thread_data_.end_pose.push_back(msg->pose.position.y);
    thread_data_.end_pose.push_back(common::math::QuaternionToHeading(
        msg->pose.orientation.w, msg->pose.orientation.x,
        msg->pose.orientation.y, msg->pose.orientation.z));
    destination_ready_.store(true);
  }
}

bool OpenSpaceTrajectoryWrapper::Proc() {
  ros::Rate loop_rate(1);
  spinner_->start();

  OpenSpaceTrajectoryThreadData thread_data;

  AINFO("waiting for data");
  while (ros::ok()) {
    if (localization_ready_.load() && obstacles_ready_.load() &&
        destination_ready_.load()) {
      data_ready_.store(true);
    }
    if (!data_ready_.load()) {
      AWARN("Input data not ready");
      loop_rate.sleep();
      continue;
    }
    AINFO("Input data received");

    {
      std::lock(obstacles_copy_done_, localization_copy_done_,
                destination_copy_done_);
      // make sure both already-locked mutexes are unlocked at the end of
      // scope
      std::lock_guard<std::timed_mutex> lock1(obstacles_copy_done_,
                                              std::adopt_lock);
      std::lock_guard<std::timed_mutex> lock2(localization_copy_done_,
                                              std::adopt_lock);
      std::lock_guard<std::timed_mutex> lock3(destination_copy_done_,
                                              std::adopt_lock);
      thread_data = thread_data_;
      localization_ready_.store(false);
      obstacles_ready_.store(false);
      destination_ready_.store(false);
      data_ready_.store(false);
    }

    GetRoiBoundary(thread_data.cur_pose[0], thread_data.cur_pose[1],
                   thread_data.cur_pose[2], thread_data.end_pose[0],
                   thread_data.end_pose[1], thread_data.end_pose[2],
                   thread_data.cur_pose[0], thread_data.cur_pose[1],
                   thread_data.cur_pose[2], &thread_data.XYbounds);

    Status status = open_space_trajectory_generator_->Plan(
        thread_data.stitching_trajectory, thread_data.cur_pose,
        thread_data.end_pose, thread_data.XYbounds, thread_data.rotate_angle,
        thread_data.translate_origin, thread_data.obstacles_vertices_vec);

    open_space_trajectory_generator_->GetOptimizedTrajectory(
        &optimized_trajectory_);

    const size_t horizon = optimized_trajectory_.size();

    std::vector<double> visual_x(horizon, 0.0);
    std::vector<double> visual_y(horizon, 0.0);
    std::vector<double> visual_phi(horizon, 0.0);

    for (size_t i = 0; i < horizon; i++) {
      const auto& point = optimized_trajectory_[i];
      visual_x[i] = point.path_point().x();
      visual_y[i] = point.path_point().y();
      visual_phi[i] = point.path_point().theta();
    }

    optimized_trajectory_markers_["arrows"] =
        optimized_trajectory_visualizer_->Arrows(visual_x, visual_y,
                                                 visual_phi);
    optimized_trajectory_markers_["points_and_lines"] =
        optimized_trajectory_visualizer_->PointsAndLines(visual_x, visual_y,
                                                         visual_phi);

    HybridAStarResult warm_start;

    open_space_trajectory_generator_->GetWarmStartResult(&warm_start);

    warm_start_markers_["arrows"] = warm_start_visualizer_->Arrows(
        warm_start.x, warm_start.y, warm_start.phi);
    warm_start_markers_["points_and_lines"] =
        warm_start_visualizer_->PointsAndLines(warm_start.x, warm_start.y,
                                               warm_start.phi);

    trajectory_updated_.store(true);
    loop_rate.sleep();
  }

  spinner_->stop();
  return true;
  // return status.ok();
}

void OpenSpaceTrajectoryWrapper::GetRoiBoundary(
    const double sx, const double sy, const double sphi, const double ex,
    const double ey, const double ephi, const double vx, const double vy,
    const double vphi, std::vector<double>* XYbounds) {
  std::vector<common::math::Vec2d> boundary_points;

  const auto vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  const auto vehicle_bbox = Node3d::GetBoundingBox(vehicle_param, vx, vy, vphi);
  const auto& vehicle_vertices = vehicle_bbox.GetAllCorners();

  boundary_points.insert(std::end(boundary_points),
                         std::begin(vehicle_vertices),
                         std::end(vehicle_vertices));

  const auto front_to_center = config_.parking_spot_config().front_to_center();
  const auto rear_to_center = config_.parking_spot_config().rear_to_center();
  const auto left_to_center = config_.parking_spot_config().left_to_center();
  const auto right_to_center = config_.parking_spot_config().right_to_center();

  auto end_point_vertices =
      CenterToRectangle(ex, ey, ephi, left_to_center, right_to_center,
                        front_to_center, rear_to_center);

  boundary_points.insert(std::end(boundary_points),
                         std::begin(end_point_vertices),
                         std::end(end_point_vertices));

  const auto default_lateral_range = config_.default_lateral_range();
  const auto default_longitudinal_range = config_.default_longitudinal_range();

  const auto search_area_vertices = CenterToRectangle(
      sx, sy, sphi, default_lateral_range / 2.0, default_lateral_range / 2.0,
      default_longitudinal_range / 2.0, default_longitudinal_range / 2.0);

  boundary_points.insert(std::end(boundary_points),
                         std::begin(search_area_vertices),
                         std::end(search_area_vertices));

  auto xminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d& a, const Vec2d& b) { return a.x() < b.x(); });
  auto yminmax = std::minmax_element(
      boundary_points.begin(), boundary_points.end(),
      [](const Vec2d& a, const Vec2d& b) { return a.y() < b.y(); });

  *XYbounds = std::vector<double>({xminmax.first->x(), xminmax.second->x(),
                                   yminmax.first->y(), yminmax.second->y()});
}

std::vector<Vec2d> OpenSpaceTrajectoryWrapper::CenterToRectangle(
    const double cx, const double cy, const double cphi, const double left,
    const double right, const double front, const double rear) {
  Vec2d center(cx, cy);

  Vec2d front_to_center(front, 0.0);
  Vec2d rear_to_center(-rear, 0.0);
  Vec2d left_to_center(0.0, left);
  Vec2d right_to_center(0.0, -right);

  front_to_center.SelfRotate(cphi);
  rear_to_center.SelfRotate(cphi);
  left_to_center.SelfRotate(cphi);
  right_to_center.SelfRotate(cphi);

  Vec2d front_center = center + front_to_center;
  Vec2d rear_center = center + rear_to_center;

  Vec2d front_left = front_center + left_to_center;
  Vec2d front_right = front_center + right_to_center;
  Vec2d rear_left = rear_center + left_to_center;
  Vec2d rear_right = rear_center + right_to_center;

  return std::vector<Vec2d>({front_left, front_right, rear_left, rear_right});
}

}  // namespace planning
}  // namespace autoagric