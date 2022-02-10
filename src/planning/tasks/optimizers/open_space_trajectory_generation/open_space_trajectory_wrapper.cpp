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

  if (!nh_.getParam("x", start_point_x_)) {
    AWARN(
        "Unable to retrived starting point x-axis value. Using default value");
  }

  if (!nh_.getParam("y", start_point_y_)) {
    AWARN(
        "Unable to retrived starting point y-axis value. Using default value");
  }

  if (!nh_.getParam("phi", start_point_phi_)) {
    AWARN(
        "Unable to retrived starting point heading angle value. Using default "
        "value");
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

  open_space_trajectory_generator_.reset(new OpenSpaceTrajectoryGenerator(
      config_.open_space_trajectory_provider_config()
          .open_space_trajectory_optimizer_config()));

  warm_start_visualizer_.reset(new common::util::TrajectoryVisualizer(nh_));

  optimized_trajectory_visualizer_.reset(
      new common::util::TrajectoryVisualizer(nh_));

  obstacle_visualizer_.reset(new common::util::TrajectoryVisualizer(nh_));

  std::unordered_map<std::string,
                     std::pair<std_msgs::ColorRGBA, geometry_msgs::Vector3>>
      markers_properties;

  std_msgs::ColorRGBA color;
  geometry_msgs::Vector3 scale;

  color.a = 1.0;
  color.r = 0.9;
  color.g = 0.9;
  color.b = 0.1;
  scale.x = 0.1;
  scale.y = 0.05;
  scale.z = 0.05;
  markers_properties["arrows"] = std::make_pair(color, scale);
  scale.x = 0.05;
  markers_properties["points_and_lines"] = std::make_pair(color, scale);
  markers_properties["boundingboxs"] = std::make_pair(color, scale);
  markers_properties["texts"] = std::make_pair(color, scale);

  warm_start_visualizer_->Setup("warm_start", "/map", markers_properties, true,
                                true, false, true, false);

  color.a = 1.0;
  color.r = 0.1;
  color.g = 1.0;
  color.b = 0.1;
  scale.x = 0.1;
  scale.y = 0.05;
  scale.z = 0.05;
  markers_properties["arrows"] = std::make_pair(color, scale);
  scale.x = 0.05;
  markers_properties["points_and_lines"] = std::make_pair(color, scale);
  markers_properties["boundingboxs"] = std::make_pair(color, scale);
  markers_properties["texts"] = std::make_pair(color, scale);

  optimized_trajectory_visualizer_->Setup(
      "optimized", "/map", markers_properties, true, true, false, true, false);

  color.a = 1.0;
  color.r = 1.0;
  color.g = 0.1;
  color.b = 0.1;
  scale.x = 0.05;
  scale.y = 0.05;
  scale.z = 0.05;
  markers_properties["obstacles"] = std::make_pair(color, scale);

  color.a = 1.0;
  color.r = 1.0;
  color.g = 0.1;
  color.b = 1.0;
  markers_properties["boundingboxs"] = std::make_pair(color, scale);
  obstacle_visualizer_->Setup("obstacle", "/map", markers_properties, true,
                              false, false, true, true);

  trajectory_marker_writer_ = std::make_unique<ros::Timer>(nh_.createTimer(
      ros::Duration(1), &OpenSpaceTrajectoryWrapper::Visualize, this));

  return true;
}

void OpenSpaceTrajectoryWrapper::Visualize(const ros::TimerEvent& e) {
  if (trajectory_updated_.load()) {
    optimized_trajectory_visualizer_->Publish(optimized_trajectory_markers_);
    warm_start_visualizer_->Publish(warm_start_markers_);
    obstacle_visualizer_->Publish(obstacle_markers_);
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
    trajectory_updated_.store(false);
  }
}

bool OpenSpaceTrajectoryWrapper::Proc() {
  ros::Rate loop_rate(1);
  spinner_->start();

  OpenSpaceTrajectoryThreadData thread_data;

  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  AINFO("waiting for data");
  while (ros::ok()) {
    if (localization_ready_.load() && obstacles_ready_.load() &&
        destination_ready_.load()) {
      data_ready_.store(true);
    }
    if (!data_ready_.load()) {
      AWARN_EVERY(10, "....");
      loop_rate.sleep();
      continue;
    }
    AINFO("Input received");

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

    thread_data.obstacles_vertices_vec.clear();

    if (!GetVirtualParkingLot(thread_data.cur_pose[0], thread_data.cur_pose[1],
                              thread_data.cur_pose[2], start_point_x_,
                              start_point_y_, start_point_phi_,
                              &thread_data.obstacles_vertices_vec,
                              &thread_data.end_pose)) {
      AERROR("Generete virtual parking lot failed");
      return false;
    }

    obstacle_markers_["obstacles"] =
        obstacle_visualizer_->BoundingBoxs(thread_data.obstacles_vertices_vec);

    obstacle_markers_["boundingboxs"] = obstacle_visualizer_->BoundingBoxs(
        {thread_data.end_pose[0]}, {thread_data.end_pose[1]},
        {thread_data.end_pose[2]}, vehicle_param);

    GetRoiBoundary(thread_data.cur_pose[0], thread_data.cur_pose[1],
                   thread_data.cur_pose[2], thread_data.end_pose[0],
                   thread_data.end_pose[1], thread_data.end_pose[2],
                   thread_data.cur_pose[0], thread_data.cur_pose[1],
                   thread_data.cur_pose[2], &thread_data.XYbounds);

    std::vector<double> roi_x = {
        thread_data.XYbounds[0], thread_data.XYbounds[0],
        thread_data.XYbounds[1], thread_data.XYbounds[1],
        thread_data.XYbounds[0]};
    std::vector<double> roi_y = {
        thread_data.XYbounds[2], thread_data.XYbounds[3],
        thread_data.XYbounds[3], thread_data.XYbounds[2],
        thread_data.XYbounds[2]};
    std::vector<double> roi_theta(roi_x.size(), 0.0);

    obstacle_markers_["points_and_lines"] =
        obstacle_visualizer_->PointsAndLines(roi_x, roi_y, roi_theta);

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
    optimized_trajectory_markers_["boundingboxs"] =
        optimized_trajectory_visualizer_->BoundingBoxs(
            visual_x, visual_y, visual_phi, vehicle_param, 10U);

    HybridAStarResult warm_start;

    open_space_trajectory_generator_->GetWarmStartResult(&warm_start);

    warm_start_markers_["arrows"] = warm_start_visualizer_->Arrows(
        warm_start.x, warm_start.y, warm_start.phi);
    warm_start_markers_["points_and_lines"] =
        warm_start_visualizer_->PointsAndLines(warm_start.x, warm_start.y,
                                               warm_start.phi);
    warm_start_markers_["boundingboxs"] = warm_start_visualizer_->BoundingBoxs(
        warm_start.x, warm_start.y, warm_start.phi, vehicle_param, 10U);

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

bool OpenSpaceTrajectoryWrapper::GetVirtualParkingLot(
    const double sx, const double sy, const double stheta, const double vx,
    const double vy, const double vtheta,
    std::vector<std::vector<Vec2d>>* obstacles_vertices_vec,
    std::vector<double>* end_pose) {
  /**
   *    y
   *    ^
   *    |------------------------------------------------
   *    |                                               |
   *    |                  OBSTACLE_4                   |
   *    |                                               |
   *    |------------------------------------------------
   *    |
   *    |
   *    |       VEHICLE(vx, vy)
   *    |
   *    |-----------------              -----------------
   *    |                | PARKING SPOT |               |
   *    |   OBSTACLE_1   |--------------|  OBSTACLE_3   |
   *    |                |  OBSTACLE_2  |               |
   * (0, 0) ----------------------------------------------->x
   */

  auto vehicle_param = common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double length = vehicle_param.length();
  const double witdh = vehicle_param.width();

  const double back_edge_to_center = vehicle_param.back_edge_to_center();

  /**
   * @todo move to conf
   */
  const double virtual_obstacle_1_length = 15.5;
  const double virtual_obstacle_2_length = 3.0;
  const double virtual_obstacle_3_length = 15.0;
  const double virtual_obstacle_4_length = virtual_obstacle_1_length +
                                           virtual_obstacle_2_length +
                                           virtual_obstacle_3_length;

  const double road_width = 15.0;
  const double virtual_obstacle_1_width = 6.5;
  const double virtual_obstacle_2_width = 1.0;
  const double virtual_obstacle_3_width = 6.5;
  const double virtual_obstacle_4_width = 3.5;
  ADEBUG("Virtual parking lot boundaries:"
         << "\nx: " << (length / 2.0) << " to "
         << (virtual_obstacle_4_length - length / 2.0)
         << "\ny: " << (virtual_obstacle_1_width + witdh / 2.0) << " to "
         << (virtual_obstacle_1_width + road_width - witdh / 2.0));

  if (vy <= (virtual_obstacle_1_width + witdh / 2.0) ||
      vy >= (virtual_obstacle_1_width + road_width - witdh / 2.0) ||
      vx <= (length / 2.0) ||
      vx >= (virtual_obstacle_4_length - length / 2.0)) {
    AERROR("Vehicle relative position outside parking lot");
    return false;
  }

  Vec2d obstacle_1_center(virtual_obstacle_1_length / 2.0,
                          virtual_obstacle_1_width / 2.0);
  Vec2d obstacle_2_center(
      virtual_obstacle_1_length + virtual_obstacle_2_length / 2.0,
      virtual_obstacle_2_width / 2.0);
  Vec2d obstacle_3_center(virtual_obstacle_1_length +
                              virtual_obstacle_2_length +
                              virtual_obstacle_3_length / 2.0,
                          virtual_obstacle_3_width / 2.0);
  Vec2d obstacle_4_center(virtual_obstacle_4_length / 2.0,
                          road_width + virtual_obstacle_4_width / 2.0);

  Vec2d end_pose_center(
      virtual_obstacle_1_length + virtual_obstacle_2_length / 2.0,
      (virtual_obstacle_1_width + virtual_obstacle_2_width) / 2.0);

  double rotate_angle = common::math::NormalizeAngle(stheta - vtheta);
  auto vehicle_position = Vec2d(sx, sy);
  vehicle_position.SelfRotate(-rotate_angle);
  Vec2d translate_vec = vehicle_position - Vec2d(vx, vy);

  obstacle_1_center += translate_vec;
  obstacle_1_center.SelfRotate(rotate_angle);
  obstacle_2_center += translate_vec;
  obstacle_2_center.SelfRotate(rotate_angle);
  obstacle_3_center += translate_vec;
  obstacle_3_center.SelfRotate(rotate_angle);
  obstacle_4_center += translate_vec;
  obstacle_4_center.SelfRotate(rotate_angle);
  end_pose_center += translate_vec;
  end_pose_center.SelfRotate(rotate_angle);

  *end_pose = {end_pose_center.x(), end_pose_center.y(), rotate_angle + M_PI_2};

  obstacles_vertices_vec->emplace_back(CenterToRectangle(
      obstacle_1_center.x(), obstacle_1_center.y(), rotate_angle,
      virtual_obstacle_1_width / 2.0, virtual_obstacle_1_width / 2.0,
      virtual_obstacle_1_length / 2.0, virtual_obstacle_1_length / 2.0));
  obstacles_vertices_vec->emplace_back(CenterToRectangle(
      obstacle_2_center.x(), obstacle_2_center.y(), rotate_angle,
      virtual_obstacle_2_width / 2.0, virtual_obstacle_2_width / 2.0,
      virtual_obstacle_2_length / 2.0, virtual_obstacle_2_length / 2.0));
  obstacles_vertices_vec->emplace_back(CenterToRectangle(
      obstacle_3_center.x(), obstacle_3_center.y(), rotate_angle,
      virtual_obstacle_3_width / 2.0, virtual_obstacle_3_width / 2.0,
      virtual_obstacle_3_length / 2.0, virtual_obstacle_3_length / 2.0));
  obstacles_vertices_vec->emplace_back(CenterToRectangle(
      obstacle_4_center.x(), obstacle_4_center.y(), rotate_angle,
      virtual_obstacle_4_width / 2.0, virtual_obstacle_4_width / 2.0,
      virtual_obstacle_4_length / 2.0, virtual_obstacle_4_length / 2.0));

  // std::copy(obstacle_1_vec.begin(), obstacle_1_vec.end(),
  //           std::back_inserter(*obstacles_vertices_vec));
  // std::copy(obstacle_2_vec.begin(), obstacle_2_vec.end(),
  //           std::back_inserter(*obstacles_vertices_vec));
  // std::copy(obstacle_3_vec.begin(), obstacle_3_vec.end(),
  //           std::back_inserter(*obstacles_vertices_vec));cat
  // std::copy(obstacle_4_vec.begin(), obstacle_4_vec.end(),
  //           std::back_inserter(*obstacles_vertices_vec));

  return true;
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

  return std::vector<Vec2d>(
      {front_left, front_right, rear_right, rear_left, front_left});
}

}  // namespace planning
}  // namespace autoagric