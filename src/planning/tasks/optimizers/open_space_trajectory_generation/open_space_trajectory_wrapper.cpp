#include "planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_wrapper.h"

#include <boost/bind.hpp>
#include <string>

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

  if (!common::util::GetProtoFromFile(open_space_trajectory_conf_file,
                                      &config_)) {
    AERROR("Unable to load control conf file: "
           << open_space_trajectory_conf_file);
    return false;
  } else {
    AINFO("Conf file: " << open_space_trajectory_conf_file << " is loaded.");
  }

  std::string obstacle_message;
  if (!nh_.getParam("obstacle_message", obstacle_message)) {
    AERROR("Unable to retrived obstacle message topic name");
    return false;
  }

  std::string localization_message;
  if (!nh_.getParam("gps_message", localization_message)) {
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

  return true;
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

  data_ready_ = localization_ready_ && obstacles_ready_ && destination_ready_;

  if (!data_ready_) {
    return;
  }

  OpenSpaceTrajectoryThreadData thread_data;

  {
    std::lock(obstacles_copy_done_, localization_copy_done_,
              destination_copy_done_);
    // make sure both already-locked mutexes are unlocked at the end of scope
    std::lock_guard<std::timed_mutex> lock1(obstacles_copy_done_,
                                            std::adopt_lock);
    std::lock_guard<std::timed_mutex> lock2(localization_copy_done_,
                                            std::adopt_lock);
    std::lock_guard<std::timed_mutex> lock3(destination_copy_done_,
                                            std::adopt_lock);

    thread_data = thread_data_;
  }

  GetRoiBoundary(thread_data.cur_pose[0], thread_data.cur_pose[1],
                 thread_data.cur_pose[2], thread_data.end_pose[0],
                 thread_data.end_pose[1], thread_data.end_pose[2],
                 &thread_data.XYbounds);

  Status status = open_space_trajectory_generator_->Plan(
      thread_data.stitching_trajectory, thread_data.cur_pose,
      thread_data.end_pose, thread_data.XYbounds, thread_data.rotate_angle,
      thread_data.translate_origin, thread_data.obstacles_vertices_vec);

  localization_ready_.store(false);
  obstacles_ready_.store(false);
  destination_ready_.store(false);
}

void OpenSpaceTrajectoryWrapper::GetRoiBoundary(
    const double sx, const double sy, const double sphi, const double ex,
    const double ey, const double ephi, std::vector<double>* XYbounds) {
  // sanity check
  CHECK_NOTNULL(XYbounds);

  std::vector<common::math::Vec2d> boundary_points;

  const auto vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  const auto& vehicle_state = injector_->vehicle_state();
  const auto vehicle_bbox =
      Node3d::GetBoundingBox(vehicle_param, vehicle_state->x(),
                             vehicle_state->y(), vehicle_state->yaw());
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