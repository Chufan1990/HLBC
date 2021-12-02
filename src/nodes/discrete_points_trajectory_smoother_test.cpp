/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/
#include "planning/reference_line/discrete_points_trajectory_smoother.h"

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <random>

#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/planning/planning.pb.h"
#include "autoagric/planning/reference_line_smoother_config.pb.h"
#include "autoware_msgs/Lane.h"
#include "autoware_msgs/Waypoint.h"
#include "common/macro.h"
#include "common/math/vec2d.h"
#include "common/util/file.h"
#include "control/common/trajectory_visualizer.h"
#include "geometry_msgs/Vector3.h"
#include "planning/common/planning_gflags.h"
#include "planning/reference_line/trajectory_smoother.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/MarkerArray.h"

namespace autoagric {
namespace planning {

using autoagric::common::TrajectoryPoint;

class DiscretePointTrajectorySmootherTest {
 public:
  DiscretePointTrajectorySmootherTest(TrajectorySmootherConfig config) {
    config_ = config;
    smoother_.reset(new DiscretePointsTrajectorySmoother(config_));
    anchor_points_.clear();
    ADCTrajectory ref_points;

    std::random_device rd;
    std::default_random_engine gen = std::default_random_engine(rd());
    std::normal_distribution<> dis{0, 0.005};

    double dt = 1.0;

    size_t num_of_points = 20;

    for (int i = 0; i < 0; ++i) {
      auto trajectory_point = ref_points.add_trajectory_point();

      trajectory_point->mutable_path_point()->set_x((i - 5) / 10.0 * dt +
                                                    dis(gen));
      trajectory_point->mutable_path_point()->set_y(-dt + dis(gen));
      trajectory_point->mutable_path_point()->set_theta(dis(gen));
      trajectory_point->mutable_path_point()->set_kappa(dis(gen));
      trajectory_point->mutable_path_point()->set_s(i * dt);

      trajectory_point->set_v(1.0);
      trajectory_point->set_a(0.0);
      trajectory_point->set_relative_time(i * dt);

      AnchorPoint anchor_point;
      anchor_point.path_point.CopyFrom(trajectory_point->path_point());
      anchor_point.lateral_bound = config_.max_lateral_boundary_bound();
      anchor_point.longitudinal_bound = config_.longitudinal_boundary_bound();
      anchor_point.enforced = i == 0                   ? true
                              : i == num_of_points - 1 ? true
                                                       : false;
      anchor_points_.push_back(anchor_point);
    }

    for (size_t i = 0; i < 20; ++i) {
      auto trajectory_point = ref_points.add_trajectory_point();

      trajectory_point->mutable_path_point()->set_x(sin(i * M_PI / 45.0) * dt +
                                                    dis(gen));
      trajectory_point->mutable_path_point()->set_y(-cos(i * M_PI / 45.0) * dt +
                                                    dis(gen));
      trajectory_point->mutable_path_point()->set_theta(
          atan2(trajectory_point->path_point().y(),
                trajectory_point->path_point().x()) +
          M_PI / 2.0 + dis(gen));
      trajectory_point->mutable_path_point()->set_kappa(dis(gen));
      trajectory_point->mutable_path_point()->set_s((i + 5) * dt);

      trajectory_point->set_v(1.0);
      trajectory_point->set_a(0.0);
      trajectory_point->set_relative_time((i + 5) * dt);

      AnchorPoint anchor_point;
      anchor_point.path_point.CopyFrom(trajectory_point->path_point());
      anchor_point.lateral_bound = config_.max_lateral_boundary_bound();
      anchor_point.longitudinal_bound = config_.longitudinal_boundary_bound();
      anchor_point.enforced = (i + 0) == 0                   ? true
                              : (i + 0) == num_of_points - 1 ? true
                                                             : false;
      anchor_points_.push_back(anchor_point);
    }

    ADEBUG("", ref_points.DebugString());

    trajectory_.reset(new ADCTrajectory(ref_points));
    vehicle_position_ =
        common::math::Vec2d(ref_points.trajectory_point(0).path_point().x(),
                            ref_points.trajectory_point(0).path_point().y());
  }

  common::math::Vec2d vehicle_position_;
  TrajectorySmootherConfig config_;
  std::unique_ptr<TrajectorySmoother> smoother_;
  std::unique_ptr<ADCTrajectory> trajectory_;
  std::vector<AnchorPoint> anchor_points_;
};

}  // namespace planning
}  // namespace autoagric

void ADCTrajectoryToLane(const autoagric::planning::ADCTrajectory& trajectory,
                         autoware_msgs::Lane& lane) {
  lane.waypoints.clear();
  for (auto point : trajectory.trajectory_point()) {
    autoware_msgs::Waypoint wp;
    wp.pose.pose.position.x = point.path_point().x();
    wp.pose.pose.position.y = point.path_point().y();
    wp.pose.pose.orientation =
        tf2::toMsg(tf2::Quaternion(0.0, 0.0, point.path_point().theta()));
    lane.waypoints.push_back(wp);
  }
}

typedef std::pair<visualization_msgs::MarkerArray,
                  visualization_msgs::MarkerArray>
    MarkerType;

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_smoother");

  ros::NodeHandle nh("~");

  autoagric::planning::TrajectorySmootherConfig trajectory_smoother_conf;
  autoagric::planning::ADCTrajectory smoothed_trajectory;

  ros::Publisher pub1 = nh.advertise<visualization_msgs::MarkerArray>(
      "unsmoothed_trajectory_points", 10);
  ros::Publisher pub2 = nh.advertise<visualization_msgs::MarkerArray>(
      "unsmoothed_trajectory_lane", 10);

  ros::Publisher pub3 = nh.advertise<visualization_msgs::MarkerArray>(
      "smoothed_trajectory_points", 10);
  ros::Publisher pub4 = nh.advertise<visualization_msgs::MarkerArray>(
      "smoothed_trajectory_lane", 10);

  ACHECK(!autoagric::common::util::GetProtoFromFile(
             FLAGS_discrete_points_smoother_config_filename,
             &trajectory_smoother_conf),
         "",
         "Unable to load control conf file: "
             << FLAGS_discrete_points_smoother_config_filename);

  ADEBUG("", trajectory_smoother_conf.DebugString());

  autoagric::planning::DiscretePointTrajectorySmootherTest tester(
      trajectory_smoother_conf);

  autoware_msgs::Lane old_lane;
  autoware_msgs::Lane new_lane;

  old_lane.header.frame_id = new_lane.header.frame_id = "map";
  old_lane.header.stamp = new_lane.header.stamp = ros::Time::now();

  ADCTrajectoryToLane(*tester.trajectory_, old_lane);
  //   ADCTrajectoryToLane(smoothed_trajectory, new_lane);

  std_msgs::ColorRGBA o_color, n_color;
  geometry_msgs::Vector3 scale;
  o_color.a = 1.0;
  o_color.r = 1.0;
  o_color.b = 0.0;
  o_color.g = 0.0;
  n_color.a = 1.0;
  n_color.r = 0.0;
  n_color.b = 0.0;
  n_color.g = 1.0;
  scale.x = 0.01;
  scale.y = 0.01;
  scale.z = 0.01;

  auto old_markers =
      autoagric::control::common::TrajectoryVisualizer::LaneToMarkerArray(
          old_lane, scale, o_color);
  auto new_markers = old_markers;

  tester.smoother_->SetAnchorPoints(tester.anchor_points_);
  bool status =
      tester.smoother_->Smooth(*tester.trajectory_, &smoothed_trajectory);
  ACHECK(!status, "", "Unable to smoother trajectory");

  ADEBUG("", "think we got it: \n" << smoothed_trajectory.DebugString());

  if (status) {
    ADCTrajectoryToLane(smoothed_trajectory, new_lane);
    new_markers =
        autoagric::control::common::TrajectoryVisualizer::LaneToMarkerArray(
            new_lane, scale, n_color);
  }
  for (int i = 0; i < old_markers.first.markers.size(); i++) {
    ADEBUG("", old_markers.first.markers[i].pose.position.x
                   << " " << old_markers.first.markers[i].pose.position.y
                   << " || " << new_markers.first.markers[i].pose.position.x
                   << " " << new_markers.first.markers[i].pose.position.y);
  }

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    pub1.publish(old_markers.first);
    pub2.publish(old_markers.second);
    if (status) {
      pub3.publish(new_markers.first);
      pub4.publish(new_markers.second);
    }
    ADEBUG("", "are we doing this?");
    loop_rate.sleep();
  }

  return 0;
}
