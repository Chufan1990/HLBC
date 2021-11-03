#include "control/common/msg_to_proto.h"

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

#include "common/macro.h"
#include "common/math/math_utils.h"

/**
 * @namespace autoagric::control
 * @brief autoagric::control
 */
namespace autoagric {
namespace control {

using autoagric::canbus::Chassis;
using autoagric::common::PathPoint;
using autoagric::common::TrajectoryPoint;
using autoagric::localization::LocalizationEstimate;
using autoagric::planning::ADCTrajectory;

namespace {
double PointDistanceSquare(const TrajectoryPoint& point, const double x,
                           const double y) {
  const double dx = point.path_point().x() - x;
  const double dy = point.path_point().y() - y;
  return dx * dx + dy * dy;
}
}  // namespace

void GetProtoFromMsg(const autoware_msgs::LaneConstPtr& msg,
                     ADCTrajectory* trajectory) {
  trajectory->Clear();
  trajectory->mutable_header()->set_timestamp_sec(msg->header.stamp.toSec());
  trajectory->mutable_header()->set_sequence_num(msg->header.seq);
  trajectory->mutable_header()->set_frame_id(msg->header.frame_id);

  for (size_t i = 0; i < std::min(msg->waypoints.size(), size_t(20)); i++) {
    auto& waypoint = msg->waypoints[i];
    auto&& trajectory_point = trajectory->add_trajectory_point();
    trajectory_point->mutable_path_point()->set_x(
        waypoint.pose.pose.position.x);
    trajectory_point->mutable_path_point()->set_y(
        waypoint.pose.pose.position.y);
    trajectory_point->mutable_path_point()->set_z(
        waypoint.pose.pose.position.z);
    trajectory_point->mutable_path_point()->set_kappa(0.0);
    trajectory_point->mutable_path_point()->set_theta(
        common::math::NormalizeAngle(tf2::getYaw(tf2::Quaternion(
            waypoint.pose.pose.orientation.x, waypoint.pose.pose.orientation.y,
            waypoint.pose.pose.orientation.z,
            waypoint.pose.pose.orientation.w))));
    trajectory_point->set_v(waypoint.twist.twist.linear.x);
  }

  //   ADEBUG("", "trajectory->trajectory_point(): " <<
  //   trajectory->DebugString());

  trajectory->mutable_header()->set_timestamp_sec(msg->header.stamp.toSec());
  trajectory->mutable_header()->set_frame_id(msg->header.frame_id);
  trajectory->mutable_header()->set_sequence_num(msg->header.seq);
}

void GetProtoFromMsg(const geometry_msgs::PoseStampedConstPtr& msg1,
                     const geometry_msgs::TwistStampedConstPtr& msg2,
                     LocalizationEstimate* localization) {
  localization->Clear();
  localization->mutable_pose()->mutable_position()->set_x(
      msg1->pose.position.x);
  localization->mutable_pose()->mutable_position()->set_y(
      msg1->pose.position.y);
  localization->mutable_pose()->mutable_position()->set_z(
      msg1->pose.position.z);
  localization->mutable_pose()->mutable_orientation()->set_qx(
      msg1->pose.orientation.x);
  localization->mutable_pose()->mutable_orientation()->set_qy(
      msg1->pose.orientation.y);
  localization->mutable_pose()->mutable_orientation()->set_qz(
      msg1->pose.orientation.z);
  localization->mutable_pose()->mutable_orientation()->set_qw(
      msg1->pose.orientation.w);
  localization->mutable_pose()->set_heading(
      common::math::NormalizeAngle(tf2::getYaw(tf2::Quaternion(
          msg1->pose.orientation.x, msg1->pose.orientation.y,
          msg1->pose.orientation.z, msg1->pose.orientation.w))));
  localization->mutable_pose()->mutable_linear_velocity()->set_x(
      msg2->twist.linear.x);
  localization->mutable_pose()->mutable_angular_velocity()->set_z(
      msg2->twist.angular.z);
  localization->mutable_pose()->mutable_angular_velocity_vrf()->set_z(
      msg2->twist.angular.z);
  localization->mutable_pose()->mutable_linear_acceleration_vrf()->set_x(0.0);
  localization->mutable_header()->set_timestamp_sec(msg1->header.stamp.toSec());
  localization->mutable_header()->set_frame_id(msg1->header.frame_id);
  localization->mutable_header()->set_sequence_num(msg1->header.seq);
}

void GetProtoFromMsg(const autoware_msgs::ControlCommandStampedConstPtr& msg,
                     Chassis* chassis) {
  chassis->Clear();
  chassis->set_steering_percentage(msg->cmd.steering_angle / 26.0);
  chassis->set_speed_mps(msg->cmd.linear_velocity / 100.0);
  chassis->set_gear_location(Chassis::GEAR_DRIVE);
  chassis->set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  chassis->mutable_header()->set_timestamp_sec(msg->header.stamp.toSec());
  chassis->mutable_header()->set_frame_id(msg->header.frame_id);
  chassis->mutable_header()->set_sequence_num(msg->header.seq);
}

void UpdateTrajectoryPoint(const LocalizationEstimate* localization,
                           ADCTrajectory* trajectory) {
  std::vector<std::pair<double, double>> xy;

  auto p0 = trajectory->trajectory_point(1);

  double init_time = 0.0;
  double init_s = 0.0;

  for (int i = 2; i < trajectory->trajectory_point_size(); i++) {
    auto&& mutable_p = trajectory->mutable_trajectory_point(i);
    auto& p1 = trajectory->trajectory_point(i);

    /**
     * @note estimate longitudinal distance (station) by Euclidean distance
     * between two consecutive points, i.e, s = sqrt((x0 - x1)^2 + (y0 - y1)^2)
     */
    const double dist_between_2_points = std::sqrt(
        PointDistanceSquare(p1, p0.path_point().x(), p0.path_point().y()));

    /**
     * @note estimate relative time (station) by Euclidean distance dividing
     * linear speed of former point, i.e, relative_time = s / v
     */
    const double relative_time_between_2_points =
        dist_between_2_points / p0.v();

    const double heading_diff =
        p1.path_point().theta() - p0.path_point().theta();

    init_s += dist_between_2_points;
    init_time += relative_time_between_2_points;

    // ADEBUG("", "dist_between_2_points: " << dist_between_2_points
    //                                      << " init_s: " << init_s);
    // ADEBUG("",
    //        "relative_time_between_2_points: " <<
    //        relative_time_between_2_points
    //                                           << " init_time: " <<
    //                                           init_time);

    mutable_p->mutable_path_point()->set_s(init_s);

    mutable_p->set_relative_time(init_time);

    mutable_p->mutable_path_point()->set_kappa(heading_diff /
                                               dist_between_2_points);

    p0 = p1;
  }

  trajectory->mutable_trajectory_point(1)->mutable_path_point()->set_s(.00);
  trajectory->mutable_trajectory_point(1)->set_relative_time(0.0);
  trajectory->mutable_trajectory_point(1)->mutable_path_point()->set_kappa(
      trajectory->trajectory_point(2).path_point().kappa());
  //   ADEBUG("", "trajectory->trajectory_point(): " <<
  //   trajectory->DebugString());
}

}  // namespace control
}  // namespace autoagric