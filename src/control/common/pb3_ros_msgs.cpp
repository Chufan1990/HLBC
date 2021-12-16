#include "control/common/pb3_ros_msgs.h"

#include <algorithm>
#include <cmath>

#include "common/macro.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "hlbc/TrajectoryPoint.h"
#include "ros/ros.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/**
 * @namespace autoagric::control
 * @brief autoagric::control
 */
namespace autoagric {
namespace control {

using autoagric::canbus::Chassis;
using autoagric::common::PathPoint;
using autoagric::common::TrajectoryPoint;
using autoagric::common::math::Vec2d;
using autoagric::localization::LocalizationEstimate;
using autoagric::planning::ADCTrajectory;

namespace {
double PointDistanceSquare(const TrajectoryPoint& point, const double x,
                           const double y) {
  const double dx = point.path_point().x() - x;
  const double dy = point.path_point().y() - y;
  return dx * dx + dy * dy;
}

double sign(const double x) { return x < 0 ? -1.0 : 1.0; }
}  // namespace

namespace pb3 {
void fromMsg(const autoware_msgs::LaneConstPtr& msg,
             ADCTrajectory* trajectory) {
  trajectory->Clear();
  trajectory->mutable_header()->set_timestamp_sec(msg->header.stamp.toSec());
  trajectory->mutable_header()->set_sequence_num(msg->header.seq);
  trajectory->mutable_header()->set_frame_id(msg->header.frame_id);

  double pv = std::fabs(msg->waypoints[0].twist.twist.linear.x) < 1e-2
                  ? sign(msg->waypoints[0].twist.twist.linear.x) * 1e-2
                  : msg->waypoints[0].twist.twist.linear.x;

  Vec2d pvec(msg->waypoints[0].pose.pose.position.x,
             msg->waypoints[0].pose.pose.position.y);
  double relative_time = 0.0;
  double distance = 0.0;

  for (int i = 0; i < std::min<int>(msg->waypoints.size(), 21); i++) {
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
    trajectory_point->mutable_path_point()->set_s(distance);
    trajectory_point->set_relative_time(relative_time);
    Vec2d nvec(msg->waypoints[i].pose.pose.position.x,
               msg->waypoints[i].pose.pose.position.y);
    distance += pvec.DistanceTo(nvec);
    relative_time += std::fabs(pvec.DistanceTo(nvec) / pv);
    pv = std::fabs(waypoint.twist.twist.linear.x) < 1e-2
             ? sign(waypoint.twist.twist.linear.x) * 1e-2
             : waypoint.twist.twist.linear.x;
  }

  //   ADEBUG("trajectory->trajectory_point(): " <<
  //   trajectory->DebugString());

  trajectory->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
  trajectory->mutable_header()->set_frame_id("map");
}

void fromMsg(const geometry_msgs::PoseStampedConstPtr& msg,
             LocalizationEstimate* localization) {
  localization->mutable_pose()->mutable_position()->set_x(msg->pose.position.x);
  localization->mutable_pose()->mutable_position()->set_y(msg->pose.position.y);
  localization->mutable_pose()->mutable_position()->set_z(msg->pose.position.z);
  localization->mutable_pose()->mutable_orientation()->set_qx(
      msg->pose.orientation.x);
  localization->mutable_pose()->mutable_orientation()->set_qy(
      msg->pose.orientation.y);
  localization->mutable_pose()->mutable_orientation()->set_qz(
      msg->pose.orientation.z);
  localization->mutable_pose()->mutable_orientation()->set_qw(
      msg->pose.orientation.w);
  localization->mutable_pose()->set_heading(
      common::math::NormalizeAngle(tf2::getYaw(
          tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                          msg->pose.orientation.z, msg->pose.orientation.w))));
  localization->mutable_header()->set_timestamp_sec(msg->header.stamp.toSec());
  localization->mutable_header()->set_frame_id(msg->header.frame_id);
  localization->mutable_header()->set_sequence_num(msg->header.seq);
}

void fromMsg(const geometry_msgs::TwistStampedConstPtr& msg,
             LocalizationEstimate* localization) {
  localization->mutable_pose()->mutable_linear_velocity()->set_x(
      msg->twist.linear.x);
  localization->mutable_pose()->mutable_linear_velocity()->set_y(
      msg->twist.linear.y);
  localization->mutable_pose()->mutable_linear_velocity()->set_z(
      msg->twist.linear.z);
  localization->mutable_pose()->mutable_angular_velocity()->set_x(
      msg->twist.angular.x);
  localization->mutable_pose()->mutable_angular_velocity()->set_y(
      msg->twist.angular.y);
  localization->mutable_pose()->mutable_angular_velocity()->set_z(
      msg->twist.angular.z);
  localization->mutable_pose()->mutable_linear_acceleration_vrf()->set_x(0.0);
  localization->mutable_pose()->mutable_linear_acceleration_vrf()->set_y(0.0);
  localization->mutable_pose()->mutable_linear_acceleration_vrf()->set_z(0.0);
  localization->mutable_pose()->mutable_angular_velocity_vrf()->set_x(
      msg->twist.angular.x);
  localization->mutable_pose()->mutable_angular_velocity_vrf()->set_y(
      msg->twist.angular.y);
  localization->mutable_pose()->mutable_angular_velocity_vrf()->set_z(
      msg->twist.angular.z);
}

void fromMsg(const autoware_msgs::ControlCommandStampedConstPtr& msg,
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

void fromMsg(const geometry_msgs::TwistStampedConstPtr& msg,
             canbus::Chassis* chassis) {
  chassis->Clear();
  chassis->set_steering_percentage(msg->twist.angular.z);
  chassis->set_speed_mps(msg->twist.linear.x);
  chassis->set_gear_location(msg->twist.linear.x >= 0 ? Chassis::GEAR_DRIVE
                                                      : Chassis::GEAR_REVERSE);
  chassis->set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  chassis->mutable_header()->set_timestamp_sec(msg->header.stamp.toSec());
  chassis->mutable_header()->set_frame_id(msg->header.frame_id);
  chassis->mutable_header()->set_sequence_num(msg->header.seq);
}

void fromMsg(const hlbc::TrajectoryConstPtr& msg,
             planning::ADCTrajectory* trajectory) {
  trajectory->Clear();
  trajectory->mutable_header()->set_timestamp_sec(msg->header.stamp.toSec());
  trajectory->mutable_header()->set_frame_id(msg->header.frame_id);
  trajectory->mutable_header()->set_sequence_num(msg->header.seq);

  for (int i = 0; i < msg->trajectory_point.size(); i++) {
    auto& msg_point = msg->trajectory_point[i];
    auto&& trajectory_point = trajectory->add_trajectory_point();

    trajectory_point->mutable_path_point()->set_x(msg_point.path_point.x);
    trajectory_point->mutable_path_point()->set_y(msg_point.path_point.y);
    trajectory_point->mutable_path_point()->set_s(msg_point.path_point.s);
    trajectory_point->mutable_path_point()->set_kappa(
        msg_point.path_point.kappa);
    trajectory_point->mutable_path_point()->set_theta(
        msg_point.path_point.theta);
    trajectory_point->set_v(msg_point.v);
    trajectory_point->set_a(msg_point.a);
    trajectory_point->set_relative_time(msg_point.relative_time);
  }
}

void fromMsg(const hlbc::Trajectory& msg, planning::ADCTrajectory* trajectory) {
  trajectory->Clear();
  trajectory->mutable_header()->set_timestamp_sec(msg.header.stamp.toSec());
  trajectory->mutable_header()->set_frame_id(msg.header.frame_id);
  trajectory->mutable_header()->set_sequence_num(msg.header.seq);

  for (int i = 0; i < msg.trajectory_point.size(); i++) {
    auto& msg_point = msg.trajectory_point[i];
    auto&& trajectory_point = trajectory->add_trajectory_point();

    trajectory_point->mutable_path_point()->set_x(msg_point.path_point.x);
    trajectory_point->mutable_path_point()->set_y(msg_point.path_point.y);
    trajectory_point->mutable_path_point()->set_s(msg_point.path_point.s);
    trajectory_point->mutable_path_point()->set_kappa(
        msg_point.path_point.kappa);
    trajectory_point->mutable_path_point()->set_theta(
        msg_point.path_point.theta);
    trajectory_point->set_v(msg_point.v);
    trajectory_point->set_a(msg_point.a);
    trajectory_point->set_relative_time(msg_point.relative_time);
  }
}

hlbc::Trajectory toMsg(const planning::ADCTrajectory& trajectory) {
  hlbc::Trajectory msg;
  msg.header.stamp.sec = std::floor(trajectory.header().timestamp_sec());
  msg.header.stamp.nsec = (trajectory.header().timestamp_sec() -
                           msg.header.stamp.sec) *
                          1e9;
  msg.header.frame_id = trajectory.header().frame_id();
  msg.header.seq = trajectory.header().sequence_num();

  for (const auto& trajectory_point : trajectory.trajectory_point()) {
    hlbc::TrajectoryPoint point;
    point.path_point.x = trajectory_point.path_point().x();
    point.path_point.y = trajectory_point.path_point().y();
    point.path_point.s = trajectory_point.path_point().s();
    point.path_point.theta = trajectory_point.path_point().theta();
    point.path_point.kappa = trajectory_point.path_point().kappa();
    point.v = trajectory_point.v();
    point.a = trajectory_point.a();
    point.relative_time = trajectory_point.relative_time();
    msg.trajectory_point.emplace_back(point);
  }
  return msg;
}

// void UpdateTrajectoryPoint(const LocalizationEstimate* localization,
//                            ADCTrajectory* trajectory) {
//   std::vector<std::pair<double, double>> xy;

//   auto p0 = trajectory->trajectory_point(1);

//   double init_time = 0.0;
//   double init_s = 0.0;

//   for (int i = 2; i < trajectory->trajectory_point_size(); i++) {
//     auto&& mutable_p = trajectory->mutable_trajectory_point(i);
//     auto& p1 = trajectory->trajectory_point(i);

//     /**
//      * @note estimate longitudinal distance (station) by Euclidean distance
//      * between two consecutive points, i.e, s = sqrt((x0 - x1)^2 + (y0 -
//      y1)^2)
//      */
//     const double dist_between_2_points = std::sqrt(
//         PointDistanceSquare(p1, p0.path_point().x(), p0.path_point().y()));

//     /**
//      * @note estimate relative time (station) by Euclidean distance dividing
//      * linear speed of former point, i.e, relative_time = s / v
//      */
//     const double relative_time_between_2_points =
//         dist_between_2_points / p0.v();

//     const double heading_diff =
//         p1.path_point().theta() - p0.path_point().theta();

//     init_s += dist_between_2_points;
//     init_time += relative_time_between_2_points;

//     mutable_p->mutable_path_point()->set_s(init_s);

//     mutable_p->set_relative_time(init_time);

//     mutable_p->mutable_path_point()->set_kappa(heading_diff /
//                                                dist_between_2_points);

//     p0 = p1;
//   }

//   trajectory->mutable_trajectory_point(1)->mutable_path_point()->set_s(.00);
//   trajectory->mutable_trajectory_point(1)->set_relative_time(0.0);
//   trajectory->mutable_trajectory_point(1)->mutable_path_point()->set_kappa(
//       trajectory->trajectory_point(2).path_point().kappa());
//   ADEBUG("updated trajectory: " << trajectory->DebugString());
// }
}  // namespace pb3
}  // namespace control
}  // namespace autoagric