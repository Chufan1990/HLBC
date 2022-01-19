#include "control/common/pb3_ros_msgs.h"

#include <algorithm>
#include <cmath>

#include "common/macro.h"
#include "common/math/math_utils.h"
#include "common/math/quaternion.h"
#include "common/math/vec2d.h"
#include "common/util/point_factory.h"
#include "hlbc/TrajectoryPoint.h"

/**
 * @namespace autoagric::control
 * @brief autoagric::control
 */
namespace autoagric {
namespace control {

using autoagric::canbus::Chassis;
using autoagric::common::util::PointFactory;
using autoagric::localization::LocalizationEstimate;
using autoagric::planning::ADCTrajectory;
using common::PathPoint;
using common::TrajectoryPoint;
using common::math::Vec2d;

namespace {
constexpr double kDoubleEpsilon = 1e-3;
}  // namespace

namespace pb3 {

void fromMsg(const std_msgs::Header& msg, common::Header* header) {
  header->set_timestamp_sec(msg.stamp.toSec());
  header->set_sequence_num(msg.seq);
  header->set_frame_id(msg.frame_id);
}

void fromMsg(const autoware_msgs::LaneConstPtr& msg,
             ADCTrajectory* trajectory) {
  trajectory->Clear();

  if (msg->waypoints.size() == 0U) return;

  double prev_x = msg->waypoints.front().pose.pose.position.x;
  double prev_y = msg->waypoints.front().pose.pose.position.y;

  double prev_heading = common::math::QuaternionToHeading(
      msg->waypoints.front().pose.pose.orientation.w,
      msg->waypoints.front().pose.pose.orientation.x,
      msg->waypoints.front().pose.pose.orientation.y,
      msg->waypoints.front().pose.pose.orientation.z);

  double prev_speed = msg->waypoints.front().twist.twist.linear.x;

  double s = 0.0;
  double relative_time = 0.0;

  for (size_t i = 0; i < msg->waypoints.size(); i++) {
    const auto& waypoint = msg->waypoints[i];
    auto trajectory_point = trajectory->add_trajectory_point();

    const double x = waypoint.pose.pose.position.x;
    const double y = waypoint.pose.pose.position.y;
    const double z = waypoint.pose.pose.position.z;
    const double heading = common::math::QuaternionToHeading(
        waypoint.pose.pose.orientation.w, waypoint.pose.pose.orientation.x,
        waypoint.pose.pose.orientation.y, waypoint.pose.pose.orientation.z);
    const double v = waypoint.twist.twist.linear.x;

    Vec2d diff(x - prev_x, y - prev_y);

    const double ds = diff.Length();
    const double dt =
        ds / std::max(std::abs(v + prev_speed) / 2.0, kDoubleEpsilon);
    const double a = (v - prev_speed) / dt;
    const double kappa =
        ds / common::math::NormalizeAngle(heading - prev_heading);

    s += ds;
    relative_time += dt;

    trajectory_point->set_v(v);
    trajectory_point->set_a(a);
    trajectory_point->set_relative_time(relative_time);
    trajectory_point->mutable_path_point()->CopyFrom(
        PointFactory::ToPathPoint(x, y, z, s, heading, kappa));

    prev_x = x;
    prev_y = y;
    prev_speed = v;
    prev_heading = heading;
  }
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
  localization->mutable_pose()->set_heading(common::math::QuaternionToHeading(
      msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z));

  fromMsg(msg->header, localization->mutable_header());
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
  fromMsg(msg->header, chassis->mutable_header());
}

void fromMsg(const geometry_msgs::TwistStampedConstPtr& msg,
             canbus::Chassis* chassis) {
  chassis->Clear();
  chassis->set_steering_percentage(msg->twist.angular.z);
  chassis->set_speed_mps(msg->twist.linear.x);
  chassis->set_gear_location(msg->twist.linear.x >= 0 ? Chassis::GEAR_DRIVE
                                                      : Chassis::GEAR_REVERSE);
  chassis->set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  fromMsg(msg->header, chassis->mutable_header());
}

void fromMsg(const hlbc::TrajectoryConstPtr& msg,
             planning::ADCTrajectory* trajectory) {
  trajectory->Clear();
  for (size_t i = 0; i < msg->trajectory_point.size(); i++) {
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
  fromMsg(msg->header, trajectory->mutable_header());
}

void fromMsg(const hlbc::Trajectory& msg, planning::ADCTrajectory* trajectory) {
  trajectory->Clear();

  for (size_t i = 0; i < msg.trajectory_point.size(); i++) {
    auto& msg_point = msg.trajectory_point[i];
    auto&& trajectory_point = trajectory->add_trajectory_point();
    fromMsg(msg_point, trajectory_point);
  }
  fromMsg(msg.header, trajectory->mutable_header());
}

hlbc::Trajectory toMsg(const planning::ADCTrajectory& trajectory) {
  hlbc::Trajectory msg;
  msg.header.stamp.sec = std::floor(trajectory.header().timestamp_sec());
  msg.header.stamp.nsec =
      (trajectory.header().timestamp_sec() - msg.header.stamp.sec) * 1e9;
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

void fromMsg(const hlbc::TrajectoryPoint& msg, common::TrajectoryPoint* point) {
  fromMsg(msg.path_point, point->mutable_path_point());
  point->set_v(msg.v);
  point->set_a(msg.a);
  point->set_relative_time(msg.relative_time);
}

void fromMsg(const hlbc::PathPoint& msg, common::PathPoint* point) {
  point->set_x(msg.x);
  point->set_y(msg.y);
  point->set_s(msg.s);
  point->set_kappa(msg.kappa);
  point->set_theta(msg.theta);
}
}  // namespace pb3
}  // namespace control
}  // namespace autoagric