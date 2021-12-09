#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <memory>

#include "autoagric/canbus/chassis.pb.h"
#include "autoagric/localization/localization.pb.h"
#include "autoagric/planning/planning.pb.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/Lane.h"
/**
 * @namespace autoagric::control
 * @brief autoagric::control
 */
namespace autoagric {
namespace control {

void GetProtoFromMsg(const autoware_msgs::LaneConstPtr& msg,
                     planning::ADCTrajectory* trajectory);

void GetProtoFromMsg(const geometry_msgs::PoseStampedConstPtr& msg,
                     localization::LocalizationEstimate* localization);

void GetProtoFromMsg(const geometry_msgs::TwistStampedConstPtr& msg,
                     localization::LocalizationEstimate* localization);

void GetProtoFromMsg(const autoware_msgs::ControlCommandStampedConstPtr& ms,
                     canbus::Chassis* chassis);

void GetProtoFromMsg(const geometry_msgs::TwistStampedConstPtr& msg,
                     canbus::Chassis* chassis);

void UpdateTrajectoryPoint(
    const localization::LocalizationEstimate* localization,
    planning::ADCTrajectory* trajectory);

}  // namespace control
}  // namespace autoagric