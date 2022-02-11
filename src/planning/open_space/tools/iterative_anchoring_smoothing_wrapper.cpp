#include "planning/open_space/tools/iterative_anchoring_smoothing_wrapper.h"

#include <nav_msgs/GridCells.h>

#include <string>

#include "common/configs/vehicle_config_helper.h"
#include "common/macro.h"
#include "common/math/box2d.h"
#include "common/math/quaternion.h"
#include "common/status/status.h"
#include "common/util/file.h"
#include "planning/open_space/coarse_trajectory_generator/node3d.h"
#include "planning/open_space/trajectory_smoother/iterative_anchoring_smoother.h"

namespace autoagric {
namespace planning {

using common::Status;
using common::math::Box2d;
using common::math::Vec2d;

IterativeAnchoringSmoothingWrapper::IterativeAnchoringSmoothingWrapper(
    ros::NodeHandle& nh)
    : nh_(nh) {
  AINFO("....");
}

IterativeAnchoringSmoothingWrapper::~IterativeAnchoringSmoothingWrapper() {
  spinner_->stop();
}

bool IterativeAnchoringSmoothingWrapper::Init() {
  std::string config_filename;
  if (!nh_.getParam("config", config_filename)) {
    AERROR(
        "Unable to retrived iterative anchoring smoothing wrapper "
        "configuration file");
    return false;
  }

  config_filename =
      absl::StrCat(std::string(std::getenv("HOME")), config_filename);

  if (!common::util::GetProtoFromFile(config_filename, &config_)) {
    AERROR("Unable to load control conf file: " << config_filename);
    return false;
  } else {
    AINFO("Configuration file " << config_filename
                                << " is loaded: " << config_.DebugString());
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

  if (!nh_.getParam("parallel_parking", parallel_parking_)) {
    AWARN(
        "Unable to retrived parallel parking value. Using default "
        "value");
  }

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

  warm_start_visualizer_->Setup("warm_start", "/map", markers_properties, true,
                                true, false, true, false);
  markers_properties.clear();

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

  optimized_trajectory_visualizer_->Setup(
      "optimized", "/map", markers_properties, true, true, false, true, false);

  markers_properties.clear();

  color.a = 1.0;
  color.r = 1.0;
  color.g = 0.1;
  color.b = 0.1;
  scale.x = 0.05;
  scale.y = 0.05;
  scale.z = 0.05;
  markers_properties["obstacles"] = std::make_pair(color, scale);

  color.a = 0.5;
  color.r = 1.0;
  color.g = 0.1;
  color.b = 1.0;
  markers_properties["boundingboxs"] = std::make_pair(color, scale);
  obstacle_visualizer_->Setup("obstacle", "/map", markers_properties, false,
                              false, false, true, true);

  spinner_ = std::make_unique<ros::AsyncSpinner>(0);

  trajectory_marker_writer_ = std::make_unique<ros::Timer>(nh_.createTimer(
      ros::Duration(5), &IterativeAnchoringSmoothingWrapper::Visualize, this));

  dp_map_writer_ = std::make_unique<ros::Publisher>(
      nh_.advertise<nav_msgs::GridCells>("dp_map", 1));

  return true;
}

void IterativeAnchoringSmoothingWrapper::Visualize(const ros::TimerEvent& e) {
  if (trajectory_updated_.load()) {
    optimized_trajectory_visualizer_->Publish(optimized_trajectory_markers_);
    warm_start_visualizer_->Publish(warm_start_markers_);
  }

  if (obstacles_updated_.load()) {
    obstacle_visualizer_->Publish(obstacle_markers_);
  }

  if (dp_map_updated_.load()) {
    dp_map_writer_->publish(dp_map_);
  }
}

bool IterativeAnchoringSmoothingWrapper::Proc() {
  spinner_->start();

  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  std::vector<double> cur_pose(3, 0.0);
  std::vector<double> end_pose(3, 0.0);
  std::vector<double> XYbounds(4, 0.0);
  std::vector<std::vector<common::math::Vec2d>> obstacles_vertices_vec;
  const double rotate_angle = start_point_phi_;
  const Vec2d translate_origin(start_point_x_, start_point_y_);

  obstacles_vertices_vec.clear();

  cur_pose[0] = start_point_x_;
  cur_pose[1] = start_point_y_;
  cur_pose[2] = start_point_phi_;

  if (!GetVirtualParkingLot(cur_pose[0], cur_pose[1], cur_pose[2],
                            &obstacles_vertices_vec, &end_pose)) {
    AERROR("Generete virtual parking lot failed");
    return false;
  }

  obstacle_markers_["obstacles"] =
      obstacle_visualizer_->BoundingBoxs(obstacles_vertices_vec);

  obstacle_markers_["boundingboxs"] = obstacle_visualizer_->BoundingBoxs(
      {end_pose[0]}, {end_pose[1]}, {end_pose[2]}, vehicle_param);

  obstacles_updated_.store(true);

  std::vector<common::math::Vec2d> boundary_points;

  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    boundary_points.insert(boundary_points.end(), obstacle_vertices.begin(),
                           obstacle_vertices.end());
  }

  GetRoiBoundary(cur_pose[0], cur_pose[1], cur_pose[2], end_pose[0],
                 end_pose[1], end_pose[2], boundary_points, &XYbounds);

  HybridAStar hybrid_astar(config_.planner_open_space_config());

  IterativeAnchoringSmoother iterative_anchoring_smoother(
      config_.planner_open_space_config());

  const auto start_timestamp = std::chrono::system_clock::now();

  double init_v = 0.0;
  double init_a = 0.0;

  // PathPointNormalizing(rotate_angle, translate_origin, &end_pose[0],
  //                      &end_pose[1], &end_pose[2]);

  HybridAStarResult warm_start_result;

  if (!hybrid_astar.Plan(cur_pose[0], cur_pose[1], cur_pose[2], end_pose[0],
                         end_pose[1], end_pose[2], XYbounds,
                         obstacles_vertices_vec, &warm_start_result)) {
    ADEBUG("Failed to solve hybrid astar search problem");
    return false;
  }

  dp_map_.header.frame_id = "map";

  dp_map_.cell_height = config_.planner_open_space_config()
                            .warm_start_config()
                            .grid_a_star_xy_resolution();
  dp_map_.cell_width = config_.planner_open_space_config()
                           .warm_start_config()
                           .grid_a_star_xy_resolution();

  for (const auto& node : hybrid_astar.DpMap()) {
    geometry_msgs::Point point;
    point.x = node.second->GetGridX() * config_.planner_open_space_config()
                                            .warm_start_config()
                                            .grid_a_star_xy_resolution() +
              XYbounds[0];
    point.y = node.second->GetGridY() * config_.planner_open_space_config()
                                            .warm_start_config()
                                            .grid_a_star_xy_resolution() +
              XYbounds[2];
    point.z = 0.0;
    dp_map_.cells.emplace_back(std::move(point));
  }

  dp_map_updated_.store(true);

  AINFO("Hybrid Astar solved");

  Eigen::MatrixXd xWS;
  Eigen::MatrixXd uWS;
  Eigen::MatrixXd state_result_ds;
  Eigen::MatrixXd control_result_ds;
  Eigen::MatrixXd time_result_ds;

  bool partition = true;

  if (partition) {
    std::vector<HybridAStarResult> partitioned_trajectories;

    if (!hybrid_astar.TrajectoryPartition(warm_start_result,
                                          &partitioned_trajectories)) {
      ADEBUG("Hybrid Astar partition failed");
      return false;
    }

    size_t size = partitioned_trajectories.size();
    std::vector<Eigen::MatrixXd> xWS_vec;
    std::vector<Eigen::MatrixXd> uWS_vec;
    std::vector<Eigen::MatrixXd> state_result_ds_vec;
    std::vector<Eigen::MatrixXd> control_result_ds_vec;
    std::vector<Eigen::MatrixXd> time_result_ds_vec;
    xWS_vec.resize(size);
    uWS_vec.resize(size);
    state_result_ds_vec.resize(size);
    control_result_ds_vec.resize(size);
    time_result_ds_vec.resize(size);

    ADEBUG("Trajectories size in smoother is " << size);
    for (size_t i = 0; i < size; i++) {
      LoadHybridAstarResultInEigen(&partitioned_trajectories[i], &xWS_vec[i],
                                   &uWS_vec[i]);

      DiscretizedTrajectory smoothed_trajectory;
      const auto smoother_start_timestamp = std::chrono::system_clock::now();
      if (!iterative_anchoring_smoother.Smooth(xWS_vec[i], init_a, init_v,
                                               obstacles_vertices_vec,
                                               &smoothed_trajectory)) {
        ADEBUG("Smoother fail at " << i << "th trajectory");
        ADEBUG(i << "th trajectory size is " << xWS_vec[i].cols());
        return false;
      }

      LoadResult(smoothed_trajectory, &state_result_ds_vec[i],
                 &control_result_ds_vec[i], &time_result_ds_vec[i]);

      const auto smoother_end_timestamp = std::chrono::system_clock::now();
      const auto smoother_diff =
          std::chrono::duration<double, std::milli>(smoother_end_timestamp -
                                                    smoother_start_timestamp)
              .count();
      ADEBUG("Partitioned trajectory smoothing total time: "
             << smoother_diff << " ms at the " << i << "th trajectory.");
      ADEBUG("The " << i << "th trajectory pre-smoothing size is "
                    << xWS_vec[i].cols() << "; post-smoothing size is "
                    << state_result_ds_vec[i].cols());
    }

    CombineTrajectories(xWS_vec, uWS_vec, state_result_ds_vec,
                        control_result_ds_vec, time_result_ds_vec, &xWS, &uWS,
                        &state_result_ds, &control_result_ds, &time_result_ds);
  }

  // rescale the states to the world frame
  // size_t state_size = state_result_ds.cols();
  // for (size_t i = 0; i < state_size; ++i) {
  //   PathPointDeNormalizing(rotate_angle, translate_origin,
  //                          &(state_result_ds(0, i)), &(state_result_ds(1,
  //                          i)),
  //                          &(state_result_ds(2, i)));
  // }

  const auto optimized_trajectory =
      LoadTrajectory(state_result_ds, control_result_ds, time_result_ds);

  const auto end_timestamp = std::chrono::system_clock::now();
  const auto diff =
      std::chrono::duration<double, std::milli>(end_timestamp - start_timestamp)
          .count();

  AINFO("Iterative anchoring smoother total time: " << diff << " ms");

  const size_t horizon = optimized_trajectory.size();

  std::vector<double> visual_x(horizon, 0.0);
  std::vector<double> visual_y(horizon, 0.0);
  std::vector<double> visual_phi(horizon, 0.0);

  for (size_t i = 0; i < horizon; i++) {
    const auto& point = optimized_trajectory[i];
    visual_x[i] = point.path_point().x();
    visual_y[i] = point.path_point().y();
    visual_phi[i] = point.path_point().theta();
  }

  optimized_trajectory_markers_["arrows"] =
      optimized_trajectory_visualizer_->Arrows(visual_x, visual_y, visual_phi);
  optimized_trajectory_markers_["points_and_lines"] =
      optimized_trajectory_visualizer_->PointsAndLines(visual_x, visual_y,
                                                       visual_phi);
  optimized_trajectory_markers_["boundingboxs"] =
      optimized_trajectory_visualizer_->BoundingBoxs(
          visual_x, visual_y, visual_phi, vehicle_param, 10U);

  warm_start_markers_["arrows"] = warm_start_visualizer_->Arrows(
      warm_start_result.x, warm_start_result.y, warm_start_result.phi);
  warm_start_markers_["points_and_lines"] =
      warm_start_visualizer_->PointsAndLines(
          warm_start_result.x, warm_start_result.y, warm_start_result.phi);
  warm_start_markers_["boundingboxs"] = warm_start_visualizer_->BoundingBoxs(
      warm_start_result.x, warm_start_result.y, warm_start_result.phi,
      vehicle_param, 10U);

  trajectory_updated_.store(true);

  ros::Rate rate(1);

  while (ros::ok()) {
    rate.sleep();
  }

  return true;
}

void IterativeAnchoringSmoothingWrapper::GetRoiBoundary(
    const double sx, const double sy, const double sphi, const double ex,
    const double ey, const double ephi,
    std::vector<common::math::Vec2d>& boundary_points,
    std::vector<double>* XYbounds) {
  const auto vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  const auto vehicle_bbox = Node3d::GetBoundingBox(vehicle_param, sx, sy, sphi);
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

bool IterativeAnchoringSmoothingWrapper::GetVirtualParkingLot(
    const double sx, const double sy, const double stheta,
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
  const double center_shift_distance = length / 2.0 - back_edge_to_center;
  const double parking_spot_heading = parallel_parking_ ? 0.0 : M_PI_2;
  const double y_axis_shift_distance =
      0.8 * (length - center_shift_distance) * std::sin(parking_spot_heading) +
      0.5 * witdh * std::cos(parking_spot_heading);

  // sanity check
  CHECK_GE(config_.virtual_parkinglot_obstacles_width().size(), 3U);
  CHECK_EQ(config_.virtual_parkinglot_obstacles_width().size(),
           config_.virtual_parkinglot_obstacles_length().size());

  /**
   * @todo move to conf
   */
  const double virtual_obstacle_1_length =
      config_.virtual_parkinglot_obstacles_length(0);
  const double virtual_obstacle_2_length =
      config_.virtual_parkinglot_obstacles_length(1);
  const double virtual_obstacle_3_length =
      config_.virtual_parkinglot_obstacles_length(2);
  const double virtual_obstacle_4_length = virtual_obstacle_1_length +
                                           virtual_obstacle_2_length +
                                           virtual_obstacle_3_length;

  const double road_width = config_.road_width();
  const double virtual_obstacle_1_width =
      config_.virtual_parkinglot_obstacles_width(0);
  const double virtual_obstacle_2_width =
      config_.virtual_parkinglot_obstacles_width(1);
  const double virtual_obstacle_3_width =
      config_.virtual_parkinglot_obstacles_width(2);
  const double virtual_obstacle_4_width = virtual_obstacle_2_width;

  if (sy <= (virtual_obstacle_1_width / 2.0) ||
      sy >= (virtual_obstacle_1_width + road_width / 2.0) || sx <= 0.0 ||
      sx >= (virtual_obstacle_4_length / 2.0)) {
    AERROR("Vehicle relative position outside parking lot");
    AERROR("Virtual parking lot boundaries:"
           << "\nx: " << 0.0 << " to " << (virtual_obstacle_4_length / 2.0)
           << "\ny: " << (virtual_obstacle_1_width / 2.0) << " to "
           << (virtual_obstacle_1_width + road_width / 2.0));
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
      virtual_obstacle_1_width - y_axis_shift_distance);

  //   double rotate_angle = common::math::NormalizeAngle(stheta - vtheta);
  //   auto vehicle_position = Vec2d(sx, sy);
  //   vehicle_position.SelfRotate(-rotate_angle);
  //   Vec2d translate_vec = vehicle_position - Vec2d(vx, vy);

  //   obstacle_1_center += translate_vec;
  //   obstacle_1_center.SelfRotate(rotate_angle);
  //   obstacle_2_center += translate_vec;
  //   obstacle_2_center.SelfRotate(rotate_angle);
  //   obstacle_3_center += translate_vec;
  //   obstacle_3_center.SelfRotate(rotate_angle);
  //   obstacle_4_center += translate_vec;
  //   obstacle_4_center.SelfRotate(rotate_angle);
  //   end_pose_center += translate_vec;
  //   end_pose_center.SelfRotate(rotate_angle);

  *end_pose = {end_pose_center.x() -
                   center_shift_distance * std::cos(parking_spot_heading),
               end_pose_center.y() -
                   center_shift_distance * std::sin(parking_spot_heading),
               parking_spot_heading};

  obstacles_vertices_vec->emplace_back(CenterToRectangle(
      obstacle_1_center.x(), obstacle_1_center.y(), 0.0,
      virtual_obstacle_1_width / 2.0, virtual_obstacle_1_width / 2.0,
      virtual_obstacle_1_length / 2.0, virtual_obstacle_1_length / 2.0));
  obstacles_vertices_vec->emplace_back(CenterToRectangle(
      obstacle_2_center.x(), obstacle_2_center.y(), 0.0,
      virtual_obstacle_2_width / 2.0, virtual_obstacle_2_width / 2.0,
      virtual_obstacle_2_length / 2.0, virtual_obstacle_2_length / 2.0));
  obstacles_vertices_vec->emplace_back(CenterToRectangle(
      obstacle_3_center.x(), obstacle_3_center.y(), 0.0,
      virtual_obstacle_3_width / 2.0, virtual_obstacle_3_width / 2.0,
      virtual_obstacle_3_length / 2.0, virtual_obstacle_3_length / 2.0));
  obstacles_vertices_vec->emplace_back(CenterToRectangle(
      obstacle_4_center.x(), obstacle_4_center.y(), 0.0,
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

std::vector<Vec2d> IterativeAnchoringSmoothingWrapper::CenterToRectangle(
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

void IterativeAnchoringSmoothingWrapper::LoadHybridAstarResultInEigen(
    HybridAStarResult* result, Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS) {
  size_t horizon = result->x.size() - 1;
  xWS->resize(4, horizon + 1);
  uWS->resize(2, horizon);
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->x.data(), horizon + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->y.data(), horizon + 1);
  Eigen::VectorXd phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->phi.data(), horizon + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->v.data(), horizon + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->steer.data(), horizon);
  Eigen::VectorXd a =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result->a.data(), horizon);
  xWS->row(0) = std::move(x);
  xWS->row(1) = std::move(y);
  xWS->row(2) = std::move(phi);
  xWS->row(3) = std::move(v);
  uWS->row(0) = std::move(steer);
  uWS->row(1) = std::move(a);
}

void IterativeAnchoringSmoothingWrapper::LoadResult(
    const DiscretizedTrajectory& discretized_trajectory,
    Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
    Eigen::MatrixXd* time_result_dc) {
  const size_t points_size = discretized_trajectory.size();
  CHECK_GT(points_size, 1U);
  *state_result_dc = Eigen::MatrixXd::Zero(4, points_size);
  *control_result_dc = Eigen::MatrixXd::Zero(2, points_size - 1);
  *time_result_dc = Eigen::MatrixXd::Zero(1, points_size - 1);

  auto& state_result = *state_result_dc;
  for (size_t i = 0; i < points_size; ++i) {
    state_result(0, i) = discretized_trajectory[i].path_point().x();
    state_result(1, i) = discretized_trajectory[i].path_point().y();
    state_result(2, i) = discretized_trajectory[i].path_point().theta();
    state_result(3, i) = discretized_trajectory[i].v();
  }

  auto& control_result = *control_result_dc;
  auto& time_result = *time_result_dc;
  const double wheel_base = common::VehicleConfigHelper::Instance()
                                ->GetConfig()
                                .vehicle_param()
                                .wheel_base();
  for (size_t i = 0; i + 1 < points_size; ++i) {
    control_result(0, i) =
        std::atan(discretized_trajectory[i].path_point().kappa() * wheel_base);
    control_result(1, i) = discretized_trajectory[i].a();
    time_result(0, i) = discretized_trajectory[i + 1].relative_time() -
                        discretized_trajectory[i].relative_time();
  }
}

void IterativeAnchoringSmoothingWrapper::CombineTrajectories(
    const std::vector<Eigen::MatrixXd>& xWS_vec,
    const std::vector<Eigen::MatrixXd>& uWS_vec,
    const std::vector<Eigen::MatrixXd>& state_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& control_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& time_result_ds_vec,
    Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS,
    Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
    Eigen::MatrixXd* time_result_ds) {
  // Repeated midway state point are not added
  size_t warm_start_state_size = 0;
  for (const auto& warm_start_state : xWS_vec) {
    warm_start_state_size += warm_start_state.cols();
  }
  warm_start_state_size -= xWS_vec.size() - 1;

  size_t warm_start_control_size = 0;
  for (const auto& warm_start_control : uWS_vec) {
    warm_start_control_size += warm_start_control.cols();
  }
  // Repeated midway state point are not added
  size_t smoothed_state_size = 0;
  for (const auto& smoothed_state : state_result_ds_vec) {
    smoothed_state_size += smoothed_state.cols();
  }
  smoothed_state_size -= state_result_ds_vec.size() - 1;

  size_t smoothed_control_size = 0;
  for (const auto& smoothed_control : control_result_ds_vec) {
    smoothed_control_size += smoothed_control.cols();
  }

  size_t time_size = 0;
  for (const auto& smoothed_time : time_result_ds_vec) {
    time_size += smoothed_time.cols();
  }

  Eigen::MatrixXd xWS_ =
      Eigen::MatrixXd::Zero(xWS_vec[0].rows(), warm_start_state_size);
  Eigen::MatrixXd uWS_ =
      Eigen::MatrixXd::Zero(uWS_vec[0].rows(), warm_start_control_size);
  Eigen::MatrixXd state_result_ds_ =
      Eigen::MatrixXd::Zero(state_result_ds_vec[0].rows(), smoothed_state_size);
  Eigen::MatrixXd control_result_ds_ = Eigen::MatrixXd::Zero(
      control_result_ds_vec[0].rows(), smoothed_control_size);
  Eigen::MatrixXd time_result_ds_ =
      Eigen::MatrixXd::Zero(time_result_ds_vec[0].rows(), time_size);

  size_t traj_size = xWS_vec.size();

  uint64_t counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t warm_start_state_cols = xWS_vec[i].cols() - 1;
    for (size_t j = 0; j < warm_start_state_cols; ++j) {
      xWS_.col(counter) = xWS_vec[i].col(j);
      ++counter;
    }
  }
  xWS_.col(counter) = xWS_vec.back().col(xWS_vec.back().cols() - 1);
  ++counter;
  CHECK_EQ(counter, warm_start_state_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t warm_start_control_cols = uWS_vec[i].cols();
    for (size_t j = 0; j < warm_start_control_cols; ++j) {
      uWS_.col(counter) = uWS_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, warm_start_control_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t smoothed_state_cols = state_result_ds_vec[i].cols() - 1;
    for (size_t j = 0; j < smoothed_state_cols; ++j) {
      state_result_ds_.col(counter) = state_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  state_result_ds_.col(counter) =
      state_result_ds_vec.back().col(state_result_ds_vec.back().cols() - 1);
  ++counter;
  CHECK_EQ(counter, smoothed_state_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t smoothed_control_cols = control_result_ds_vec[i].cols();
    for (size_t j = 0; j < smoothed_control_cols; ++j) {
      control_result_ds_.col(counter) = control_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, smoothed_control_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t time_cols = time_result_ds_vec[i].cols();
    for (size_t j = 0; j < time_cols; ++j) {
      time_result_ds_.col(counter) = time_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, time_size);

  *xWS = std::move(xWS_);
  *uWS = std::move(uWS_);
  *state_result_ds = std::move(state_result_ds_);
  *control_result_ds = std::move(control_result_ds_);
  *time_result_ds = std::move(time_result_ds_);
}

DiscretizedTrajectory IterativeAnchoringSmoothingWrapper::LoadTrajectory(
    const Eigen::MatrixXd& state_result, const Eigen::MatrixXd& control_result,
    const Eigen::MatrixXd& time_result) {
  DiscretizedTrajectory optimized_trajectory;

  // Optimizer doesn't take end condition control state into consideration for
  // now
  size_t states_size = state_result.cols();
  size_t times_size = time_result.cols();
  size_t controls_size = control_result.cols();
  CHECK_EQ(states_size, times_size + 1);
  CHECK_EQ(states_size, controls_size + 1);
  double relative_time = 0.0;
  double relative_s = 0.0;
  Vec2d last_path_point(state_result(0, 0), state_result(1, 0));
  for (size_t i = 0; i < states_size; ++i) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(state_result(0, i));
    point.mutable_path_point()->set_y(state_result(1, i));
    point.mutable_path_point()->set_theta(state_result(2, i));
    point.set_v(state_result(3, i));
    Vec2d cur_path_point(state_result(0, i), state_result(1, i));
    relative_s += cur_path_point.DistanceTo(last_path_point);
    point.mutable_path_point()->set_s(relative_s);
    // TODO(Jinyun): Evaluate how to set end states control input
    if (i == controls_size) {
      point.set_steer(0.0);
      point.set_a(0.0);
    } else {
      point.set_steer(control_result(0, i));
      point.set_a(control_result(1, i));
    }

    if (i == 0) {
      point.set_relative_time(relative_time);
    } else {
      relative_time += time_result(0, i - 1);
      point.set_relative_time(relative_time);
    }

    optimized_trajectory.emplace_back(std::move(point));
    last_path_point = cur_path_point;
  }

  return optimized_trajectory;
}

void IterativeAnchoringSmoothingWrapper::PathPointNormalizing(
    const double rotate_angle, const Vec2d& translate_origin, double* x,
    double* y, double* phi) {
  *x -= translate_origin.x();
  *y -= translate_origin.y();
  double tmp_x = *x;
  *x = (*x) * std::cos(-rotate_angle) - (*y) * std::sin(-rotate_angle);
  *y = tmp_x * std::sin(-rotate_angle) + (*y) * std ::cos(-rotate_angle);
  *phi = common::math::NormalizeAngle(*phi - rotate_angle);
}

void IterativeAnchoringSmoothingWrapper::PathPointDeNormalizing(
    const double rotate_angle, const Vec2d& translate_origin, double* x,
    double* y, double* phi) {
  double tmp_x = *x;
  *x = (*x) * std::cos(rotate_angle) - (*y) * std::sin(rotate_angle);
  *y = tmp_x * std::sin(rotate_angle) + (*y) * std::cos(rotate_angle);
  *x += translate_origin.x();
  *y += translate_origin.y();
  *phi = common::math::NormalizeAngle(*phi + rotate_angle);
}

}  // namespace planning
}  // namespace autoagric