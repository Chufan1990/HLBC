#include "control/control_component.h"

#include <chrono>

#include "common/macro.h"
#include "common/util/file.h"
#include "control/common/control_gflags.h"
#include "control/common/pb3_ros_msgs.h"
#include "control/controller/mpc_controller.h"
#include "geometry_msgs/Vector3.h"
#include "planning/common/planning_gflags.h"
#include "planning/reference_line/discrete_points_trajectory_smoother.h"
#include "std_msgs/ColorRGBA.h"

namespace autoagric {
namespace control {

using common::ErrorCode;
using common::Status;

ControlComponent::ControlComponent(ros::NodeHandle& nh) : nh_(nh) {}

bool ControlComponent::Init() {
  spinner_ = std::make_unique<ros::AsyncSpinner>(0);

  chassis_reader_ = std::make_unique<ros::Subscriber>(nh_.subscribe(
      FLAGS_chassis_message_name, FLAGS_chassis_pending_queue_size,
      &ControlComponent::OnChassis, this));

  // planning_reader_ = std::make_unique<ros::Subscriber>(nh_.subscribe(
  //     FLAGS_planning_message_name, FLAGS_planning_pending_queue_size,
  //     &ControlComponent::OnPlanning, this));

  planning_test_reader_ = std::make_unique<ros::Subscriber>(nh_.subscribe(
      "/static_trajectory", FLAGS_planning_pending_queue_size,
      &ControlComponent::OnPlanningTest, this));

  localization_reader_ = std::make_unique<ros::Subscriber>(nh_.subscribe(
      FLAGS_localization_message_name, FLAGS_localization_pending_queue_size,
      &ControlComponent::OnLocalization, this));

  imu_reader_ = std::make_unique<ros::Subscriber>(
      nh_.subscribe(FLAGS_imu_message_name, FLAGS_imu_pending_queue_size,
                    &ControlComponent::OnIMU, this));

  control_cmd_writer_ = std::make_unique<ros::Publisher>(
      nh_.advertise<geometry_msgs::TwistStamped>(
          FLAGS_control_cmd_message_name,
          FLAGS_control_cmd_pending_queue_size));

  ADEBUG("FLAGS_enable_trajectory_visualizer: "
         << FLAGS_enable_trajectory_visualizer);

  if (FLAGS_enable_trajectory_visualizer) {
    auto visual_nh = ros::NodeHandle(nh_, "visual");
    std::vector<std::string> names = {"resampled_trajectory",
                                      "warmstart_solution"};
    visualizer_ = std::make_unique<TrajectoryVisualizer>(visual_nh, names);
    visualizer_->Init();
  }

  injector_ = std::make_shared<DependencyInjector>();

  AERROR_IF(
      !common::util::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_),
      "Unable to load control conf file: " << FLAGS_control_conf_file);

  AERROR_IF(!common::util::GetProtoFromFile(
                FLAGS_discrete_points_smoother_config_filename,
                &trajectory_smoother_conf_),
            "Unable to load control conf file: "
                << FLAGS_discrete_points_smoother_config_filename);

  AINFO("Conf file: " << FLAGS_control_conf_file << " is loaded.");

  ADEBUG(
      "FLAGS_enable_trajectory_smoother: " << FLAGS_enable_trajectory_smoother);

  if (FLAGS_enable_trajectory_smoother) {
    smoother_ = std::unique_ptr<planning::DiscretePointsTrajectorySmoother>(
        new planning::DiscretePointsTrajectorySmoother(
            trajectory_smoother_conf_));
  }

  if (!controller_agent_.Init(injector_, &control_conf_).ok()) return false;

  ADEBUG("Control component init done");
  return true;
}

Status ControlComponent::CheckInput(LocalView* local_view) {
  if ((!local_view->has_trajectory()) ||
      local_view->trajectory().trajectory_point().empty()) {
    AWARN_EVERY(100, "planning has no trajectory point. ");

    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "planning has no trajectory point. planning_seq_num:" +
                      local_view->trajectory().header().sequence_num());
  }

  for (auto& trajectory_point :
       *local_view->mutable_trajectory()->mutable_trajectory_point()) {
    if (std::fabs(trajectory_point.v()) <
            control_conf_.minimum_speed_resolution() &&
        std::fabs(trajectory_point.a()) <
            control_conf_.max_acceleration_when_stopped()) {
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
    }
  }

  auto status = injector_->vehicle_state()->Update(local_view->localization(),
                                                   local_view->chassis());

  return status;
}

Status ControlComponent::CheckTimestamp(const LocalView& local_view) {
  if (!control_conf_.enable_input_timestamp_check() ||
      control_conf_.is_control_test_mode()) {
    ADEBUG("Skip input timestamp check by gflags");
    return Status::OK();
  }

  double current_timestamp = ros::Time::now().toSec();

  double localization_diff =
      current_timestamp - local_view.localization().header().timestamp_sec();
  if (localization_diff > (control_conf_.max_localization_miss_num() *
                           control_conf_.localization_period())) {
    AERROR("localization msg lost for " << std::setprecision(6)
                                        << localization_diff << "s");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "localization msg timeout");
  }

  double chassis_diff =
      current_timestamp - local_view.chassis().header().timestamp_sec();
  if (chassis_diff >
      (control_conf_.max_chassis_miss_num() * control_conf_.chassis_period())) {
    AERROR("chassis msg lost for " << std::setprecision(6) << chassis_diff
                                   << "s");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - local_view.trajectory().header().timestamp_sec();
  if (trajectory_diff > (control_conf_.max_planning_miss_num() *
                         control_conf_.trajectory_period())) {
    AERROR("trajectory msg lost for " << std::setprecision(6) << trajectory_diff
                                      << "s");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "trajectory msg timeout");
  }
  return Status::OK();
}

Status ControlComponent::ProduceControlCommand(
    ControlCommand* control_command) {
  local_view_.Clear();
  {
    std::lock(trajectory_copy_done_, localization_copy_done_,
              chassis_copy_done_);
    // make sure both already-locked mutexes are unlocked at the end of scope
    std::lock_guard<std::timed_mutex> lock1(trajectory_copy_done_,
                                            std::adopt_lock);
    std::lock_guard<std::timed_mutex> lock2(localization_copy_done_,
                                            std::adopt_lock);
    std::lock_guard<std::timed_mutex> lock3(chassis_copy_done_,
                                            std::adopt_lock);

    local_view_.mutable_trajectory()->Swap(&latest_trajectory_);
    local_view_.mutable_localization()->Swap(&latest_localization_);
    local_view_.mutable_chassis()->Swap(&latest_chassis_);
  }

  // ADEBUG(local_view_.DebugString());

  Status status = CheckInput(&local_view_);
  estop_ = false;
  if (!status.ok()) {
    AERROR("Control input data failed: " << status.error_message());
    estop_ = true;
    estop_reason_ = status.error_message();
  } else {
    Status status_ts = CheckTimestamp(local_view_);
    if (!status_ts.ok()) {
      AERROR("Input message timeout");
      status = status_ts;
    }
  }

  if (status.ok()) {
    ADEBUG("Ready to execuate mpc controller");
    Status status_compute = controller_agent_.ComputeControlCommand(
        &local_view_.localization(), &local_view_.chassis(),
        &local_view_.trajectory(), control_command);

    if (!status_compute.ok()) {
      AERROR("Control main function failed"
             << " with localization: "
             << local_view_.localization().ShortDebugString()
             << " with chassis: " << local_view_.chassis().ShortDebugString()
             << " with trajectory: "
             << local_view_.trajectory().ShortDebugString()
             << " with cmd: " << control_command->ShortDebugString()
             << " status:" << status_compute.error_message());
      estop_ = true;
      estop_reason_ = status_compute.error_message();
      status = status_compute;
    }
  }

  if (estop_) {
    AWARN_EVERY(100, "Estop triggered! No control core method executed!");
    // set Estop command
    control_command->set_speed(0);
    control_command->set_throttle(0);
    control_command->set_brake(control_conf_.soft_estop_brake());
    control_command->set_gear_location(canbus::Chassis::GEAR_DRIVE);
  }

  return status;
}

bool ControlComponent::Proc() {
  spinner_->start();

  geometry_msgs::Vector3 scale_1, scale_2;
  std_msgs::ColorRGBA color_1, color_2;
  if (FLAGS_enable_trajectory_visualizer) {
    scale_1.x = 0.2;
    scale_1.y = 0.2;
    color_1.r = 1.0;
    color_1.g = 0.0;
    color_1.b = 0.0;
    color_1.a = 1.0;
    scale_2.x = 0.2;
    scale_2.y = 0.2;
    color_2.r = 0.0;
    color_2.g = 1.0;
    color_2.b = 0.5;
    color_2.a = 1.0;
  }

  ControlCommand control_command;

  ros::Rate loop_rate(FLAGS_control_cmd_frequency);

  while (ros::ok()) {
    const auto start_time = ros::Time::now();
    Status status = ProduceControlCommand(&control_command);
    AERROR_IF(!status.ok(),
              "Failed to produce control command:" << status.error_message());

    ADEBUG("\n" << control_command.DebugString());

    geometry_msgs::TwistStamped cmd;

    cmd.header.stamp = ros::Time::now();
    cmd.twist.linear.x = control_command.brake() > 1e-6
                             ? 0.0
                             : std::fabs(control_command.speed());
    cmd.twist.linear.y = control_command.brake();
    cmd.twist.linear.z = control_command.gear_location();
    cmd.twist.angular.z = control_command.steering_target();
    control_cmd_writer_->publish(cmd);

    if (FLAGS_enable_trajectory_visualizer) {
      std::vector<std::pair<visualization_msgs::MarkerArray,
                            visualization_msgs::MarkerArray>>
          markers;

      const auto& controller = controller_agent_.controller();

      markers.emplace_back(
          std::move(TrajectoryVisualizer::TrajectoryToMarkerArray<
                    std::vector<common::TrajectoryPoint>>(
              controller->resampled_trajectory(),
              local_view_.trajectory().header().frame_id(),
              ros::Time::now().toSec(), scale_1, color_1)));
      markers.emplace_back(
          std::move(TrajectoryVisualizer::TrajectoryToMarkerArray<
                    std::vector<common::TrajectoryPoint>>(
              controller->warmstart_solution(),
              local_view_.trajectory().header().frame_id(),
              ros::Time::now().toSec(), scale_2, color_2)));

      visualizer_->Proc(markers);
    }
    const auto end_time = ros::Time::now();
    const double time_diff_ms = (end_time - start_time).toSec() * 1e3;
    ADEBUG("total control time spend: " << time_diff_ms << " ms.");
    loop_rate.sleep();
  }

  spinner_->stop();
  return true;
}

void ControlComponent::OnChassis(
    const geometry_msgs::TwistStampedConstPtr& msg) {
  std::unique_lock<std::timed_mutex> locker(chassis_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    pb3::fromMsg(msg, &latest_chassis_);
  }
}

void ControlComponent::OnPlanning(const autoware_msgs::LaneConstPtr& msg) {
  std::unique_lock<std::timed_mutex> locker(trajectory_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    pb3::fromMsg(msg, &latest_trajectory_);
  }
}

void ControlComponent::OnPlanningTest(const hlbc::TrajectoryConstPtr& msg) {
  std::unique_lock<std::timed_mutex> locker(trajectory_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    pb3::fromMsg(*msg, &latest_trajectory_);
  } else {
    AWARN("Giving up new trajectory");
  }
}

void ControlComponent::OnLocalization(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::timed_mutex> locker(localization_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    pb3::fromMsg(msg, &latest_localization_);
  }
}

void ControlComponent::OnIMU(const geometry_msgs::TwistStampedConstPtr& msg) {
  std::unique_lock<std::timed_mutex> locker(localization_copy_done_,
                                            std::defer_lock);
  if (locker.try_lock_for(std::chrono::milliseconds(40))) {
    pb3::fromMsg(msg, &latest_localization_);
  }
}

}  // namespace control
}  // namespace autoagric