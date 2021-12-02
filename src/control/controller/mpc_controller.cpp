#include "control/controller/mpc_controller.h"

#include "absl/strings/str_cat.h"
#include "autoagric/common/error_code.pb.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/macro.h"
#include "common/math/math_utils.h"
#include "common/math/quaternion.h"

namespace autoagric {
namespace control {

using common::ErrorCode;
using common::Status;
using Matrix = Eigen::MatrixXd;

MPCController::MPCController() : name_("MPC-based controller") {
  AINFO("", "Using " << name_);
}

MPCController::~MPCController() {}

Status MPCController::Init(std::shared_ptr<DependencyInjector> injector,
                           const ControlConf *control_conf) {
  control_conf_ = control_conf;
  injector_ = injector;
  // Load controller configuration
  if (!LoadControlConf(control_conf_)) {
    AERROR("controller/lat_controller, LatController::Init",
           "failed to load control conf");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "faild to load control_conf");
  }

  matrix_r_ = Matrix::Identity(controls_, controls_);
  matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);

  int r_param_size = control_conf->mpc_controller_conf().matrix_r_size();
  for (int i = 0; i < r_param_size; ++i) {
    matrix_r_(i, i) = control_conf->mpc_controller_conf().matrix_r(i);
  }

  int q_param_size = control_conf->mpc_controller_conf().matrix_q_size();
  if (basic_state_size_ != q_param_size) {
    const auto error_msg =
        absl::StrCat("MPC controller error: matrix_q size: ", q_param_size,
                     " in parameter file not equal to basic_state_size_: ",
                     basic_state_size_);
    AERROR("", error_msg);
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, error_msg);
  }
  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf->mpc_controller_conf().matrix_q(i);
  }
  // Update matrix_q_updated_ and matrix_r_updated_
  matrix_r_updated_ = matrix_r_;
  matrix_q_updated_ = matrix_q_;

  const size_t n_vars =
      horizon_ * basic_state_size_ + (horizon_ - 1) * controls_;
  const size_t n_constraints = horizon_ * basic_state_size_;

  x_start_ = 0;
  y_start_ = x_start_ + horizon_;
  heading_start_ = y_start_ + horizon_;
  speed_start_ = heading_start_ + horizon_;
  steer_start_ = speed_start_ + horizon_;
  accel_start_ = steer_start_ + horizon_ - 1;

  vars_ = Dvector(n_vars);
  vars_lowerbound_ = Dvector(n_vars);
  vars_upperbound_ = Dvector(n_vars);
  constraints_lowerbound_ = Dvector(n_constraints);
  constraints_upperbound_ = Dvector(n_constraints);

  for (size_t i = 0; i < n_vars; i++) vars_[i] = 0.0;

  for (size_t i = 0; i < n_vars; i++) {
    vars_lowerbound_[i] = -1e19;
    vars_upperbound_[i] = 1e19;
  }

  for (size_t i = speed_start_; i < steer_start_; i++) {
    vars_lowerbound_[i] = max_backward_speed_;
    vars_upperbound_[i] = max_forward_speed_;
  }

  for (size_t i = steer_start_; i < accel_start_; i++) {
    vars_lowerbound_[i] = -steer_single_direction_max_degree_ / 180.0 * M_PI;
    vars_upperbound_[i] = steer_single_direction_max_degree_ / 180.0 * M_PI;
  }

  for (size_t i = accel_start_; i < n_vars; i++) {
    vars_lowerbound_[i] = max_driving_deceleration_;
    vars_upperbound_[i] = max_driving_acceleration_;
  }

  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound_[i] = 0;
    constraints_upperbound_[i] = 0;
  }

  resampled_trajectory_ = std::vector<common::TrajectoryPoint>(horizon_);

  warm_up_solution_ = std::vector<common::TrajectoryPoint>(horizon_);

  LoadMPCGainScheduler(control_conf->mpc_controller_conf());

  mpc_ipopt_solver_ = std::make_shared<common::math::MpcIpopt>(
      new common::math::MpcIpopt(ipopt_options_, horizon_, ts_, lf_,
                                 control_conf->mpc_controller_conf()));
  ADEBUG("", "[MPCController] init done!");
  return Status::OK();
}

bool MPCController::LoadControlConf(const ControlConf *control_conf) {
  if (!control_conf) {
    AERROR("", "control_conf = nullptr");
    return false;
  }
  vehicle_param_ =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();

  ts_ = control_conf->mpc_controller_conf().ts();
  if (ts_ <= 0.0) {
    AERROR("", "invalid control update interval");
    return false;
  }

  cf_ = control_conf->mpc_controller_conf().cf();
  cr_ = control_conf->mpc_controller_conf().cr();

  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() / M_PI * 180;

  static constexpr double kEpsilon = 1e-6;
  if (std::isnan(steer_ratio_) || steer_ratio_ < kEpsilon) {
    AERROR("", "[MPCController] teer_ratio = 0");
    return false;
  }

  const double mass_fl = control_conf->mpc_controller_conf().mass_fl();
  const double mass_fr = control_conf->mpc_controller_conf().mass_fr();
  const double mass_rl = control_conf->mpc_controller_conf().mass_rl();
  const double mass_rr = control_conf->mpc_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  const double mpc_eps = control_conf->mpc_controller_conf().eps();
  const double mpc_max_iteration =
      control_conf->mpc_controller_conf().max_iteration();
  const double print_level = control_conf->mpc_controller_conf().print_level();

  ipopt_options_ = absl::StrCat(
      "Integer print_level ", print_level, "\n", "Sparse true forward\n",
      "Sparse true reverse\n", "Integer max_iter ", mpc_max_iteration, "\n",
      "Numeric tol ", mpc_eps, "\n");

  throttle_lowerbound_ =
      std::max(vehicle_param_.throttle_deadzone(),
               control_conf->mpc_controller_conf().throttle_minimum_action());
  brake_lowerbound_ =
      std::max(vehicle_param_.brake_deadzone(),
               control_conf->mpc_controller_conf().brake_minimum_action());

  minimum_speed_protection_ = control_conf->minimum_speed_protection();

  max_forward_speed_ = control_conf->maximum_forward_speed();
  max_backward_speed_ = control_conf->maximum_backward_speed();

  max_driving_acceleration_ = control_conf->maximum_driving_acceleration();
  max_driving_deceleration_ = control_conf->maximum_driving_deceleration();

  const double latency_time =
      control_conf->mpc_controller_conf().latency_time();
  latency_steps_ = latency_time / ts_;

  max_acceleration_when_stopped_ =
      control_conf->max_acceleration_when_stopped();
  max_abs_speed_when_stopped_ = vehicle_param_.max_abs_speed_when_stopped();
  standstill_acceleration_ =
      control_conf->mpc_controller_conf().standstill_acceleration();

  enable_mpc_feedforward_compensation_ =
      control_conf->mpc_controller_conf().enable_mpc_feedforward_compensation();

  unconstrained_control_diff_limit_ =
      control_conf->mpc_controller_conf().unconstrained_control_diff_limit();

  LoadControlCalibrationTable(control_conf->mpc_controller_conf());
  ADEBUG("", "MPC conf loaded");
  return true;
}

void MPCController::LoadMPCGainScheduler(
    const MPCControllerConf &mpc_controller_conf) {
  const auto &lat_err_gain_scheduler =
      mpc_controller_conf.lat_err_gain_scheduler();
  const auto &heading_err_gain_scheduler =
      mpc_controller_conf.heading_err_gain_scheduler();
  const auto &feedforwardterm_gain_scheduler =
      mpc_controller_conf.feedforwardterm_gain_scheduler();
  const auto &steer_weight_gain_scheduler =
      mpc_controller_conf.steer_weight_gain_scheduler();
  ADEBUG("", "MPC control gain scheduler loaded");
  Interpolation1D::DataType xy1, xy2, xy3, xy4;
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : feedforwardterm_gain_scheduler.scheduler()) {
    xy3.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : steer_weight_gain_scheduler.scheduler()) {
    xy4.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  lat_err_interpolation_.reset(new Interpolation1D);
  ACHECK(lat_err_interpolation_->Init(xy1), "",
         "Fail to load lateral error gain scheduler for MPC controller");

  heading_err_interpolation_.reset(new Interpolation1D);
  ACHECK(heading_err_interpolation_->Init(xy2), "",
         "Fail to load heading error gain scheduler for MPC controller");

  feedforwardterm_interpolation_.reset(new Interpolation1D);
  ACHECK(feedforwardterm_interpolation_->Init(xy3), "",
         "Fail to load feed forward term gain scheduler for MPC controller");

  steer_weight_interpolation_.reset(new Interpolation1D);
  ACHECK(steer_weight_interpolation_->Init(xy4), "",
         "Fail to load steer weight gain scheduler for MPC controller");
}

void MPCController::LoadControlCalibrationTable(
    const MPCControllerConf &mpc_controller_conf) {
  const auto &control_table = mpc_controller_conf.calibration_table();
  ADEBUG("", "Control calibration table loaded");
  ADEBUG("", "Control calibration table size is "
                 << control_table.calibration_size());
  Interpolation2D::DataType xyz;
  for (const auto &calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  control_interpolation_.reset(new Interpolation2D);
  ACHECK(control_interpolation_->Init(xyz), "",
         "Fail to load control calibration table");
}

Status MPCController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {
  auto target_tracking_trajectory = *planning_published_trajectory;

  auto time_stamp_diff =
      planning_published_trajectory->header().timestamp_sec() -
      current_trajectory_timestamp_;

  if (time_stamp_diff > 1e-2) {
    trajectory_analyzer_ =
        std::move(TrajectoryAnalyzer(&target_tracking_trajectory));
  }

  current_trajectory_timestamp_ =
      planning_published_trajectory->header().timestamp_sec();

  auto pos_trajectory_time_stamp_diff =
      localization->header().timestamp_sec() - current_trajectory_timestamp_;

  trajectory_analyzer_.SampleByRelativeTime(pos_trajectory_time_stamp_diff, ts_,
                                            horizon_, resampled_trajectory_);

  UpdateState();

  double mpc_start_timestamp = ros::Time::now().toNSec();

  mpc_ipopt_solver_->Update(&vars_, &vars_lowerbound_, &vars_upperbound_,
                            &constraints_lowerbound_, &constraints_upperbound_,
                            &matrix_q_updated_, &matrix_r_updated_,
                            &resampled_trajectory_);

  if (common::math::MpcIpopt::Solve(mpc_ipopt_solver_, warm_up_solution_)) {
    cmd->set_steering_target(warm_up_solution_[1 + latency_steps_].steer());
    cmd->set_speed(warm_up_solution_[1 + latency_steps_].v());
    cmd->set_acceleration(warm_up_solution_[1 + latency_steps_].a());
    ADEBUG("", "MPC ipopt problem failed! ";);
  } else {
    cmd->set_speed(0);
    cmd->set_acceleration(0);
    AERROR("", "MPC ipopt problem failed! ";);
  }

  double mpc_end_timestamp = ros::Time::now().toNSec();
  ADEBUG("", "MPC core algorithm: calculation time is: "
                 << (mpc_end_timestamp - mpc_start_timestamp) * 1000 << " ms.");

  /**
   * @todo(chufan) add emergency stop
   * @todo(chufan) add standstill acceleration
   * @todo(chufan) add calibration table for speed to throttle
   * @todo(chufan) verify the necessarity of feedforward steering compensation
   * @todo(chufan) add steering angle and speed and acceleration saturation
   */
  return Status::OK();
}

void MPCController::UpdateState() {
  const auto &com = injector_->vehicle_state()->ComputeCOMPosition(lr_);
  vars_[x_start_] = com.x();
  vars_lowerbound_[x_start_] = com.x();
  vars_upperbound_[x_start_] = com.x();
  vars_[y_start_] = com.y();
  vars_lowerbound_[y_start_] = com.y();
  vars_upperbound_[y_start_] = com.y();

  const double v = std::max(injector_->vehicle_state()->linear_velocity(),
                            minimum_speed_protection_);
  vars_[speed_start_] = v;
  vars_lowerbound_[speed_start_] = v;
  vars_upperbound_[speed_start_] = v;

  double heading = 0.0;
  const auto &orientation = injector_->vehicle_state()->pose().orientation();
  if (injector_->vehicle_state()->pose().has_heading()) {
    heading = injector_->vehicle_state()->pose().heading();
  } else {
    heading = common::math::QuaternionToHeading(
        orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
  }

  vars_[heading_start_] = heading;
  vars_lowerbound_[heading_start_] = heading;
  vars_upperbound_[heading_start_] = heading;

  const double steer =
      SteerPct2Wheel(injector_->vehicle_state()->steering_percentage());
  vars_[steer_start_] = steer;
  vars_lowerbound_[steer_start_] = steer;
  vars_upperbound_[steer_start_] = steer;

  const double accel = injector_->vehicle_state()->linear_acceleration();
  vars_[accel_start_] = accel;
  vars_lowerbound_[accel_start_] = accel;
  vars_upperbound_[accel_start_] = accel;

  for (int i = 1; i <= latency_steps_; i++) {
    vars_[steer_start_ + i] = steer;
    vars_lowerbound_[steer_start_ + i] = steer;
    vars_upperbound_[steer_start_ + i] = steer;
    vars_[accel_start_ + i] = accel;
    vars_lowerbound_[accel_start_ + i] = accel;
    vars_upperbound_[accel_start_ + i] = accel;
  }

  constraints_lowerbound_[x_start_] = com.x();
  constraints_lowerbound_[y_start_] = com.y();
  constraints_lowerbound_[heading_start_] = heading;
  constraints_lowerbound_[speed_start_] = v;

  constraints_upperbound_[x_start_] = com.x();
  constraints_upperbound_[y_start_] = com.y();
  constraints_upperbound_[heading_start_] = heading;
  constraints_upperbound_[speed_start_] = v;

  /**
   * @todo(chufan) add warm up initial guess for mpc solver
   *
   */
}

Status MPCController::Reset() { return Status::OK(); }

void MPCController::Stop() {
  // CloseLogFile();
}

std::string MPCController::Name() const { return name_; }

}  // namespace control
}  // namespace autoagric