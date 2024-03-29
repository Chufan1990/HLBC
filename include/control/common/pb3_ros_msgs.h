#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>

#include <memory>

#include "autoagric/canbus/chassis.pb.h"
#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/localization/localization.pb.h"
#include "autoagric/planning/planning.pb.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/Lane.h"
#include "hlbc/PathPoint.h"
#include "hlbc/Trajectory.h"
#include "hlbc/TrajectoryPoint.h"
/**
 * @namespace autoagric::control
 * @brief autoagric::control
 */
namespace autoagric {
namespace control {
namespace pb3 {

void fromMsg(const std_msgs::Header& msg, common::Header* header);

void fromMsg(const autoware_msgs::LaneConstPtr& msg,
             planning::ADCTrajectory* trajectory);

void fromMsg(const geometry_msgs::PoseStampedConstPtr& msg,
             localization::LocalizationEstimate* localization);

void fromMsg(const geometry_msgs::TwistStampedConstPtr& msg,
             localization::LocalizationEstimate* localization);

void fromMsg(const autoware_msgs::ControlCommandStampedConstPtr& ms,
             canbus::Chassis* chassis);

void fromMsg(const geometry_msgs::TwistStampedConstPtr& msg,
             canbus::Chassis* chassis);

void fromMsg(const hlbc::TrajectoryConstPtr& msg,
             planning::ADCTrajectory* trajectory);

void fromMsg(const hlbc::Trajectory& msg, planning::ADCTrajectory* trajectory);

void fromMsg(const hlbc::TrajectoryPoint& msg, common::TrajectoryPoint* point);

void fromMsg(const hlbc::PathPoint& msg, common::PathPoint* point);

hlbc::Trajectory toMsg(const planning::ADCTrajectory& trajectory);

void UpdateTrajectoryPoint(
    const localization::LocalizationEstimate* localization,
    planning::ADCTrajectory* trajectory);
}  // namespace pb3
}  // namespace control
}  // namespace autoagric