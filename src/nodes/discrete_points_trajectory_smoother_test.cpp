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

#include <iostream>
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

    double x[] = {22.9154, 22.6051, 22.2469, 21.8781, 21.5538, 21.2006, 20.8397,
                  20.4673, 20.1051, 19.7181, 19.3029, 18.8873, 18.5136, 18.1289,
                  17.7234, 17.3429, 16.9880, 16.6294, 16.2794, 15.9326, 15.5953,
                  15.2764, 14.9590, 14.6552, 14.3600, 14.0742, 13.8112, 13.5341,
                  13.2843, 13.0314, 12.8239, 12.6344, 12.4292, 12.2642, 12.0936,
                  11.9489, 11.8256, 11.7117, 11.6113, 11.5162, 11.4526, 11.4198,
                  11.4002, 11.3644, 11.3642, 11.3501, 11.3552};

    double y[] = {-1.2595, -1.2374, -1.2315, -1.2037, -1.1863, -1.1851, -1.1886,
                  -1.1836, -1.1584, -1.1545, -1.1622, -1.1800, -1.1673, -1.1740,
                  -1.1852, -1.1992, -1.2147, -1.2609, -1.3200, -1.3830, -1.4831,
                  -1.5895, -1.7195, -1.8631, -2.0210, -2.2148, -2.3845, -2.6029,
                  -2.8145, -3.0929, -3.3100, -3.5714, -3.8474, -4.1185, -4.4277,
                  -4.7308, -5.0062, -5.2956, -5.6418, -6.0021, -6.3546, -6.6569,
                  -7.0008, -7.3342, -7.6408, -7.9444, -8.2684};

    double z[] = {0.1881,  0.2207, 0.1987, 0.217,  0.2029, 0.1823, 0.183,
                  0.1801,  0.1739, 0.1563, 0.1566, 0.1407, 0.1445, 0.1269,
                  0.1237,  0.1439, 0.121,  0.1102, 0.0717, 0.0835, 0.087,
                  0.0911,  0.0821, 0.0867, 0.068,  0.0634, 0.0529, 0.0429,
                  0.0552,  0.0359, 0.0667, 0.0087, 0.03,   0.0189, 0.0007,
                  -0.0099, 0.0184, 0.0187, 0.0209, 0.0372, 0.0451, 0.0446,
                  0.0108,  0.0297, 0.029,  0.0339, 0.0214

    };

    double heading[] = {
        -0.0111, -0.0098, -0.0072, -0.0042, -0.0003, 0.0045, 0.0079, 0.0120,
        0.0156,  0.0202,  0.0266,  0.0356,  0.0464,  0.0603, 0.0817, 0.1092,
        0.1420,  0.1805,  0.2266,  0.2784,  0.3325,  0.3893, 0.4497, 0.5125,
        0.5753,  0.6371,  0.6980,  0.7595,  0.8199,  0.8801, 0.9350, 0.9841,
        1.0415,  1.0900,  1.1439,  1.1976,  1.2486,  1.2977, 1.3512, 1.4162,
        1.4725,  1.5164,  1.5522,  1.5708,  1.5749,  1.5761, 1.5779};

    double v[] = {0.0,
                  -0.838472385686068,
                  -1.55732428658783,
                  -2.15326618199125,
                  -2.63992955540193,
                  -3.02998237191656,
                  -3.33516089610752,
                  -3.56630150990728,
                  -3.73337253049300,
                  -3.84550602817097,
                  -3.91102964426118,
                  -3.93749840898176,
                  -3.89981935698453,
                  -3.84720490615446,
                  -3.77866597149924,
                  -3.69837179599549,
                  -3.60990991882508,
                  -3.51631799325958,
                  -3.42011560454480,
                  -3.32333608778530,
                  -3.22755834582889,
                  -3.13393866715112,
                  -3.04324254373983,
                  -2.95587648897964,
                  -2.87191985553640,
                  -2.79115665324180,
                  -2.71310736697781,
                  -2.63706077456121,
                  -2.56210576462806,
                  -2.48716315451829,
                  -2.41101750816010,
                  -2.33234895395459,
                  -2.24976500266013,
                  -2.16183236527698,
                  -2.06710877093177,
                  -1.96417478476198,
                  -1.85166562580043,
                  -1.72830298485985,
                  -1.59292684241738,
                  -1.44452728649900,
                  -1.28227633056414,
                  -1.10555973139013,
                  -0.914008806956701,
                  -0.707532254330509,
                  -0.486347967549679,
                  -0.251014855508225,
                  -0.00246465984068056};

    std::random_device rd;
    std::default_random_engine gen = std::default_random_engine(rd());
    std::normal_distribution<> dis{0, 0.005};

    double dt = 1.0;

    size_t num_of_points = sizeof(x) / sizeof(x[0]);
    ;

    for (int i = 0; i < num_of_points; ++i) {
      auto trajectory_point = ref_points.add_trajectory_point();

      trajectory_point->mutable_path_point()->set_x(x[i]);
      trajectory_point->mutable_path_point()->set_y(y[i]);
      trajectory_point->mutable_path_point()->set_z(z[i]);
      trajectory_point->mutable_path_point()->set_theta(heading[i]);
      trajectory_point->mutable_path_point()->set_kappa(0);
      trajectory_point->mutable_path_point()->set_s(0);

      trajectory_point->set_v(v[i]);
      trajectory_point->set_a(0.0);
      trajectory_point->set_relative_time(0);

      AnchorPoint anchor_point;
      anchor_point.path_point.CopyFrom(trajectory_point->path_point());
      anchor_point.lateral_bound = config_.max_lateral_boundary_bound();
      anchor_point.longitudinal_bound = config_.longitudinal_boundary_bound();
      anchor_point.enforced = i == 0                   ? true
                              : i == num_of_points - 1 ? true
                                                       : false;
      anchor_points_.push_back(anchor_point);
    }

    // for (size_t i = 0; i < 20; ++i) {
    //   auto trajectory_point = ref_points.add_trajectory_point();

    //   trajectory_point->mutable_path_point()->set_x(sin(i * M_PI / 45.0) * dt
    //   +
    //                                                 dis(gen));
    //   trajectory_point->mutable_path_point()->set_y(-cos(i * M_PI / 45.0) *
    //   dt +
    //                                                 dis(gen));
    //   trajectory_point->mutable_path_point()->set_theta(
    //       atan2(trajectory_point->path_point().y(),
    //             trajectory_point->path_point().x()) +
    //       M_PI / 2.0 + dis(gen));
    //   trajectory_point->mutable_path_point()->set_kappa(dis(gen));
    //   trajectory_point->mutable_path_point()->set_s((i + 5) * dt);

    //   trajectory_point->set_v(1.0);
    //   trajectory_point->set_a(0.0);
    //   trajectory_point->set_relative_time((i + 5) * dt);

    //   AnchorPoint anchor_point;
    //   anchor_point.path_point.CopyFrom(trajectory_point->path_point());
    //   anchor_point.lateral_bound = config_.max_lateral_boundary_bound();
    //   anchor_point.longitudinal_bound =
    //   config_.longitudinal_boundary_bound(); anchor_point.enforced = (i + 0)
    //   == 0                   ? true
    //                           : (i + 0) == num_of_points - 1 ? true
    //                                                          : false;
    //   anchor_points_.push_back(anchor_point);
    // }

    ADEBUG(ref_points.DebugString());

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

    auto q = tf2::Quaternion();
    q.setRPY(0, 0, point.path_point().theta());
    wp.pose.pose.orientation = tf2::toMsg(q);

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

  AERROR_IF(!autoagric::common::util::GetProtoFromFile(
                FLAGS_discrete_points_smoother_config_filename,
                &trajectory_smoother_conf),
            "Unable to load control conf file: "
                << FLAGS_discrete_points_smoother_config_filename);

  ADEBUG(trajectory_smoother_conf.DebugString());

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
      autoagric::control::common::TrajectoryVisualizer::toMarkerArray(
          old_lane, scale, o_color);
  auto new_markers = old_markers;

  tester.smoother_->SetAnchorPoints(tester.anchor_points_);
  bool status =
      tester.smoother_->Smooth(*tester.trajectory_, &smoothed_trajectory);
  AERROR_IF(!status, "Unable to smoother trajectory");

  ADEBUG("think we got it: \n" << smoothed_trajectory.DebugString());

  for (auto p : smoothed_trajectory.trajectory_point()) {
    std::cout << p.path_point().x() << ", " << p.path_point().y() << ", "
              << p.path_point().z() << ", " << p.path_point().theta() << ", "
              << p.v() * 1.3 << ", " << p.path_point().kappa() << " "
              << p.path_point().dkappa() << " " << p.path_point().s()
              << std::endl;
  }

  if (status) {
    ADCTrajectoryToLane(smoothed_trajectory, new_lane);
    new_markers =
        autoagric::control::common::TrajectoryVisualizer::toMarkerArray(
            new_lane, scale, n_color);
  }
  for (int i = 0; i < old_markers.first.markers.size(); i++) {
    ADEBUG(old_markers.first.markers[i].pose.position.x
           << " " << old_markers.first.markers[i].pose.position.y << " || "
           << new_markers.first.markers[i].pose.position.x << " "
           << new_markers.first.markers[i].pose.position.y);
  }

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    pub1.publish(old_markers.first);
    pub2.publish(old_markers.second);
    if (status) {
      pub3.publish(new_markers.first);
      pub4.publish(new_markers.second);
    }
    ADEBUG("are we doing this?");
    loop_rate.sleep();
  }

  return 0;
}
