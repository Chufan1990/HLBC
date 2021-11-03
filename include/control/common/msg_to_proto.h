#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <memory>

#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/Lane.h"
#include "hlbc/proto/chassis.pb.h"
#include "hlbc/proto/localization.pb.h"
#include "hlbc/proto/planning.pb.h"
/**
 * @namespace autoagric::control
 * @brief autoagric::control
 */
namespace autoagric {
namespace control {

void GetProtoFromMsg(const autoware_msgs::LaneConstPtr& msg,
                     planning::ADCTrajectory* trajectory);

void GetProtoFromMsg(const geometry_msgs::PoseStampedConstPtr& msg1,
                     const geometry_msgs::TwistStampedConstPtr& msg2,
                     localization::LocalizationEstimate* localization);

void GetProtoFromMsg(const autoware_msgs::ControlCommandStampedConstPtr& ms,
                     canbus::Chassis* chassis);

void UpdateTrajectoryPoint(
    const localization::LocalizationEstimate* localization,
    planning::ADCTrajectory* trajectory);

}  // namespace control
}  // namespace autoagric