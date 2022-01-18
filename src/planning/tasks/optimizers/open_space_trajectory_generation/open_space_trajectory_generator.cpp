
#include "planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_generator.h"

#include <chrono>
#include <iomanip>

#include "autoagric/common/error_code.pb.h"
#include "common/math/math_utils.h"

namespace autoagric {
namespace planning {

using common::ErrorCode;
using common::Status;
using common::math::Vec2d;

OpenSpaceTrajectoryGenerator::OpenSpaceTrajectoryGenerator(
    const OpenSpaceTrajectoryGeneratorConfig& config) {
  config_ = config;

  warm_start_.reset(new HybridAStar(config.planner_open_space_config()));

  iterative_anchoring_smoother_.reset(
      new IterativeAnchoringSmoother(config.planner_open_space_config()));
}

Status OpenSpaceTrajectoryGenerator::Plan(
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
    const std::vector<double>& cur_pose, const std::vector<double>& end_pose,
    const std::vector<double>& XYbounds, const double rotate_angle,
    const Vec2d& translate_origin,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec) {
  if (XYbounds.empty() || end_pose.empty() || cur_pose.empty()) {
    ADEBUG("OpenSpaceTrajectoryGenerator input data not ready");
    return Status(ErrorCode::PLANNING_ERROR,
                  "OpenSpaceTrajectoryGenerator input data not ready");
  }

  const auto start_timestamp = std::chrono::system_clock::now();

  //   stitching_trajectory_ = stitching_trajectory;

  double init_x = 0.0;
  double init_y = 0.0;
  double init_phi = 0.0;
  double init_v = 0.0;
  double init_a = 0.0;

  if (!stitching_trajectory.empty()) {
    common::TrajectoryPoint stitching_point = stitching_trajectory.back();
    init_x = stitching_point.path_point().x();
    init_y = stitching_point.path_point().y();
    init_phi = stitching_point.path_point().theta();
    init_v = stitching_point.v();
    init_a = stitching_point.a();
    ADEBUG("init x: " << std::setprecision(9) << init_x);
    ADEBUG("init y: " << std::setprecision(9) << init_y);
  } else {
    init_x = cur_pose[0];
    init_y = cur_pose[1];
    init_phi = cur_pose[2];
  }

  // Rotate and scale the state
  PathPointNormalizing(rotate_angle, translate_origin, &init_x, &init_y,
                       &init_phi);

  double end_x = end_pose[0];
  double end_y = end_pose[1];
  double end_phi = end_pose[2];
  // So far we assume the end pose is in the same frame of initial pose
  PathPointNormalizing(rotate_angle, translate_origin, &end_x, &end_y,
                       &end_phi);

  if (warm_start_->Plan(init_x, init_y, init_phi, end_x, end_y, end_phi,
                        XYbounds, obstacles_vertices_vec,
                        &warm_start_result_)) {
    ADEBUG("Warm start problem solved successfully");
  } else {
    ADEBUG("Fail to solve warm start problem");
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to solve warm start problem");
  }

  Eigen::MatrixXd xWS;
  Eigen::MatrixXd uWS;
  Eigen::MatrixXd state_result_ds;
  Eigen::MatrixXd control_result_ds;
  Eigen::MatrixXd time_result_ds;

  std::vector<HybridAStarResult> paritioned_trajectories;
  if (!warm_start_->TrajectoryPartition(warm_start_result_,
                                        &paritioned_trajectories)) {
    return Status(ErrorCode::PLANNING_ERROR, "Hybrid Astar partition failed");
  }

  size_t size = paritioned_trajectories.size();
  std::vector<Eigen::MatrixXd> xWS_vec;
  std::vector<Eigen::MatrixXd> uWS_vec;
  std::vector<Eigen::MatrixXd> state_result_ds_vec;
  std::vector<Eigen::MatrixXd> control_result_ds_vec;
  std::vector<Eigen::MatrixXd> time_result_ds_vec;
  xWS_vec.resize(size);
  uWS_vec.resize(size);
  state_result_ds_vec.resize(size);
  control_result_ds_vec.resize(size);
  time_result_ds_vec.resize(size);

  ADEBUG("Trajectories size in smoother is " << size);
  for (size_t i = 0; i < size; i++) {
    LoadHybridAstarResultInEigen(&paritioned_trajectories[i], &xWS_vec[i],
                                 &uWS_vec[i]);

    DiscretizedTrajectory smoothed_trajectory;
    const auto smoother_start_timestamp = std::chrono::system_clock::now();
    if (!GenerateDecoupledTraj(xWS_vec[i], init_a, init_v,
                               obstacles_vertices_vec, &state_result_ds_vec[i],
                               &control_result_ds_vec[i],
                               &time_result_ds_vec[i])) {
      ADEBUG("Smoother fail at " << i << "th trajectory");
      ADEBUG(i << "th trajectory size is " << xWS_vec[i].cols());
      return Status(ErrorCode::PLANNING_ERROR,
                    "Fail to solve iterative anchoring smoothing problem");
    }
    const auto smoother_end_timestamp = std::chrono::system_clock::now();
    const auto smoother_diff =
        std::chrono::duration<double, std::milli>(smoother_end_timestamp -
                                                  smoother_start_timestamp)
            .count();
    ADEBUG("Open space trajectory smoothing total time: "
           << smoother_diff << " ms at the " << i << "th trajectory.");
    ADEBUG("The " << i << "th trajectory pre-smoothing size is "
                  << xWS_vec[i].cols() << "; post-smoothing size is "
                  << state_result_ds_vec[i].cols());
  }

  CombineTrajectories(xWS_vec, uWS_vec, state_result_ds_vec,
                      control_result_ds_vec, time_result_ds_vec, &xWS, &uWS,
                      &state_result_ds, &control_result_ds, &time_result_ds);

  // rescale the states to the world frame
  size_t state_size = state_result_ds.cols();
  for (size_t i = 0; i < state_size; ++i) {
    PathPointDeNormalizing(rotate_angle, translate_origin,
                           &(state_result_ds(0, i)), &(state_result_ds(1, i)),
                           &(state_result_ds(2, i)));
  }

  LoadTrajectory(state_result_ds, control_result_ds, time_result_ds);

  const auto end_timestamp = std::chrono::system_clock::now();
  const auto diff =
      std::chrono::duration<double, std::milli>(end_timestamp - start_timestamp)
          .count();

  ADEBUG("Open space trajectory smoother total time: " << diff << " ms");
  return Status::OK();
}

void OpenSpaceTrajectoryGenerator::LoadHybridAstarResultInEigen(
    HybridAStarResult* result, Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS) {
  size_t horizon = result->x.size() - 1;
  xWS->resize(4, horizon + 1);
  uWS->resize(2, horizon);
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->x.data(), horizon + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->y.data(), horizon + 1);
  Eigen::VectorXd phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->phi.data(), horizon + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->v.data(), horizon + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result->steer.data(), horizon);
  Eigen::VectorXd a =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result->a.data(), horizon);
  xWS->row(0) = std::move(x);
  xWS->row(1) = std::move(y);
  xWS->row(2) = std::move(phi);
  xWS->row(3) = std::move(v);
  uWS->row(0) = std::move(steer);
  uWS->row(1) = std::move(a);
}

void OpenSpaceTrajectoryGenerator::PathPointNormalizing(
    const double rotate_angle, const Vec2d& translate_origin, double* x,
    double* y, double* phi) {
  *x -= translate_origin.x();
  *y -= translate_origin.y();
  double tmp_x = *x;
  *x = (*x) * std::cos(-rotate_angle) - (*y) * std::sin(-rotate_angle);
  *y = tmp_x * std::sin(-rotate_angle) + (*y) * std ::cos(-rotate_angle);
  *phi = common::math::NormalizeAngle(*phi - rotate_angle);
}

void OpenSpaceTrajectoryGenerator::PathPointDeNormalizing(
    const double rotate_angle, const Vec2d& translate_origin, double* x,
    double* y, double* phi) {
  double tmp_x = *x;
  *x = (*x) * std::cos(rotate_angle) - (*y) * std::sin(rotate_angle);
  *y = tmp_x * std::sin(rotate_angle) + (*y) * std::cos(rotate_angle);
  *x += translate_origin.x();
  *y += translate_origin.y();
  *phi = common::math::NormalizeAngle(*phi + rotate_angle);
}

bool OpenSpaceTrajectoryGenerator::GenerateDecoupledTraj(
    const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
    Eigen::MatrixXd* time_result_dc) {
  DiscretizedTrajectory smoothed_trajectory;
  if (!iterative_anchoring_smoother_->Smooth(
          xWS, init_a, init_v, obstacles_vertices_vec, &smoothed_trajectory)) {
    return false;
  }

  LoadResult(smoothed_trajectory, state_result_dc, control_result_dc,
             time_result_dc);
  return true;
}

void OpenSpaceTrajectoryGenerator::LoadResult(
    const DiscretizedTrajectory& discretized_trajectory,
    Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
    Eigen::MatrixXd* time_result_dc) {
  const size_t points_size = discretized_trajectory.size();
  CHECK_GT(points_size, 1U);
  *state_result_dc = Eigen::MatrixXd::Zero(4, points_size);
  *control_result_dc = Eigen::MatrixXd::Zero(2, points_size - 1);
  *time_result_dc = Eigen::MatrixXd::Zero(1, points_size - 1);

  auto& state_result = *state_result_dc;
  for (size_t i = 0; i < points_size; ++i) {
    state_result(0, i) = discretized_trajectory[i].path_point().x();
    state_result(1, i) = discretized_trajectory[i].path_point().y();
    state_result(2, i) = discretized_trajectory[i].path_point().theta();
    state_result(3, i) = discretized_trajectory[i].v();
  }

  auto& control_result = *control_result_dc;
  auto& time_result = *time_result_dc;
  const double wheel_base = common::VehicleConfigHelper::Instance()
                                ->GetConfig()
                                .vehicle_param()
                                .wheel_base();
  for (size_t i = 0; i + 1 < points_size; ++i) {
    control_result(0, i) =
        std::atan(discretized_trajectory[i].path_point().kappa() * wheel_base);
    control_result(1, i) = discretized_trajectory[i].a();
    time_result(0, i) = discretized_trajectory[i + 1].relative_time() -
                        discretized_trajectory[i].relative_time();
  }
}

void OpenSpaceTrajectoryGenerator::CombineTrajectories(
    const std::vector<Eigen::MatrixXd>& xWS_vec,
    const std::vector<Eigen::MatrixXd>& uWS_vec,
    const std::vector<Eigen::MatrixXd>& state_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& control_result_ds_vec,
    const std::vector<Eigen::MatrixXd>& time_result_ds_vec,
    Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS,
    Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
    Eigen::MatrixXd* time_result_ds) {
  // Repeated midway state point are not added
  size_t warm_start_state_size = 0;
  for (const auto& warm_start_state : xWS_vec) {
    warm_start_state_size += warm_start_state.cols();
  }
  warm_start_state_size -= xWS_vec.size() - 1;

  size_t warm_start_control_size = 0;
  for (const auto& warm_start_control : uWS_vec) {
    warm_start_control_size += warm_start_control.cols();
  }
  // Repeated midway state point are not added
  size_t smoothed_state_size = 0;
  for (const auto& smoothed_state : state_result_ds_vec) {
    smoothed_state_size += smoothed_state.cols();
  }
  smoothed_state_size -= state_result_ds_vec.size() - 1;

  size_t smoothed_control_size = 0;
  for (const auto& smoothed_control : control_result_ds_vec) {
    smoothed_control_size += smoothed_control.cols();
  }

  size_t time_size = 0;
  for (const auto& smoothed_time : time_result_ds_vec) {
    time_size += smoothed_time.cols();
  }

  Eigen::MatrixXd xWS_ =
      Eigen::MatrixXd::Zero(xWS_vec[0].rows(), warm_start_state_size);
  Eigen::MatrixXd uWS_ =
      Eigen::MatrixXd::Zero(uWS_vec[0].rows(), warm_start_control_size);
  Eigen::MatrixXd state_result_ds_ =
      Eigen::MatrixXd::Zero(state_result_ds_vec[0].rows(), smoothed_state_size);
  Eigen::MatrixXd control_result_ds_ = Eigen::MatrixXd::Zero(
      control_result_ds_vec[0].rows(), smoothed_control_size);
  Eigen::MatrixXd time_result_ds_ =
      Eigen::MatrixXd::Zero(time_result_ds_vec[0].rows(), time_size);

  size_t traj_size = xWS_vec.size();

  uint64_t counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t warm_start_state_cols = xWS_vec[i].cols() - 1;
    for (size_t j = 0; j < warm_start_state_cols; ++j) {
      xWS_.col(counter) = xWS_vec[i].col(j);
      ++counter;
    }
  }
  xWS_.col(counter) = xWS_vec.back().col(xWS_vec.back().cols() - 1);
  ++counter;
  CHECK_EQ(counter, warm_start_state_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t warm_start_control_cols = uWS_vec[i].cols();
    for (size_t j = 0; j < warm_start_control_cols; ++j) {
      uWS_.col(counter) = uWS_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, warm_start_control_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t smoothed_state_cols = state_result_ds_vec[i].cols() - 1;
    for (size_t j = 0; j < smoothed_state_cols; ++j) {
      state_result_ds_.col(counter) = state_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  state_result_ds_.col(counter) =
      state_result_ds_vec.back().col(state_result_ds_vec.back().cols() - 1);
  ++counter;
  CHECK_EQ(counter, smoothed_state_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t smoothed_control_cols = control_result_ds_vec[i].cols();
    for (size_t j = 0; j < smoothed_control_cols; ++j) {
      control_result_ds_.col(counter) = control_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, smoothed_control_size);

  counter = 0;
  for (size_t i = 0; i < traj_size; ++i) {
    // leave out the last repeated point so set column minus one
    uint64_t time_cols = time_result_ds_vec[i].cols();
    for (size_t j = 0; j < time_cols; ++j) {
      time_result_ds_.col(counter) = time_result_ds_vec[i].col(j);
      ++counter;
    }
  }
  CHECK_EQ(counter, time_size);

  *xWS = std::move(xWS_);
  *uWS = std::move(uWS_);
  *state_result_ds = std::move(state_result_ds_);
  *control_result_ds = std::move(control_result_ds_);
  *time_result_ds = std::move(time_result_ds_);
}

void OpenSpaceTrajectoryGenerator::LoadTrajectory(
    const Eigen::MatrixXd& state_result, const Eigen::MatrixXd& control_result,
    const Eigen::MatrixXd& time_result) {
  optimized_trajectory_.clear();

  // Optimizer doesn't take end condition control state into consideration for
  // now
  size_t states_size = state_result.cols();
  size_t times_size = time_result.cols();
  size_t controls_size = control_result.cols();
  CHECK_EQ(states_size, times_size + 1);
  CHECK_EQ(states_size, controls_size + 1);
  double relative_time = 0.0;
  double relative_s = 0.0;
  Vec2d last_path_point(state_result(0, 0), state_result(1, 0));
  for (size_t i = 0; i < states_size; ++i) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(state_result(0, i));
    point.mutable_path_point()->set_y(state_result(1, i));
    point.mutable_path_point()->set_theta(state_result(2, i));
    point.set_v(state_result(3, i));
    Vec2d cur_path_point(state_result(0, i), state_result(1, i));
    relative_s += cur_path_point.DistanceTo(last_path_point);
    point.mutable_path_point()->set_s(relative_s);
    // TODO(Jinyun): Evaluate how to set end states control input
    if (i == controls_size) {
      point.set_steer(0.0);
      point.set_a(0.0);
    } else {
      point.set_steer(control_result(0, i));
      point.set_a(control_result(1, i));
    }

    if (i == 0) {
      point.set_relative_time(relative_time);
    } else {
      relative_time += time_result(0, i - 1);
      point.set_relative_time(relative_time);
    }

    optimized_trajectory_.emplace_back(point);
    last_path_point = cur_path_point;
  }
}

}  // namespace planning
}  // namespace autoagric