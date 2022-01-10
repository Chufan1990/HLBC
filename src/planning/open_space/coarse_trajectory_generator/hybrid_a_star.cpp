#include "planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

#include <algorithm>
#include <chrono>
#include <limits>

#include "common/math/box2d.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/speed/speed_data.h"
#include "planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace autoagric {
namespace planning {

using autoagric::common::math::Box2d;
using autoagric::common::math::Vec2d;

HybridAStar::HybridAStar(const PlannerOpenSpaceConfig& open_space_conf) {
  planner_open_space_config_.CopyFrom(open_space_conf);
  reeds_shepp_generator_ =
      std::make_unique<ReedsShepp>(vehicle_param_, planner_open_space_config_);
  grid_a_star_heuristic_generator_ =
      std::make_unique<GridSearch>(planner_open_space_config_);
  next_node_num_ =
      planner_open_space_config_.warm_start_config().next_node_num();
  max_steer_angle_ =
      vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio();
  step_size_ = planner_open_space_config_.warm_start_config().step_size();
  xy_grid_resolution_ =
      planner_open_space_config_.warm_start_config().xy_grid_resolution();
  delta_t_ = planner_open_space_config_.delta_t();
  traj_forward_penalty_ =
      planner_open_space_config_.warm_start_config().traj_forward_penalty();
  traj_backward_penalty_ =
      planner_open_space_config_.warm_start_config().traj_backward_penalty();
  traj_gear_switch_penalty_ =
      planner_open_space_config_.warm_start_config().traj_gear_switch_penalty();
  traj_steer_penalty_ =
      planner_open_space_config_.warm_start_config().traj_steer_penalty();
  traj_steer_change_penalty_ = planner_open_space_config_.warm_start_config()
                                   .traj_steer_change_penalty();
}

bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<ReedsSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedsSheppPath>();
  if (!reeds_shepp_generator_->ShortestRSP(current_node, end_node_,
                                           reeds_shepp_to_check)) {
    ADEBUG("ShortestRSP fialed");
    return false;
  }

  if (!RSPCheck(reeds_shepp_to_check)) {
    return false;
  }

  ADEBUG("Reach the end configuration with Reeds Shepp");
  final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
  return true;
}

bool HybridAStar::RSPCheck(std::shared_ptr<ReedsSheppPath> reeds_shepp_to_end) {
  std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  return ValidityCheck(node);
}

bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node) {
  CHECK_NOTNULL(node);
  CHECK_GT(node->GetStepSize(), 0U);

  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }

  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // Tge first (x, y, phi) is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size != 1) {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; i++) {
    if (traversed_x[i] < XYbounds_[0] || traversed_x[i] > XYbounds_[1] ||
        traversed_y[i] < XYbounds_[2] || traversed_y[i] > XYbounds_[3]) {
      return false;
    }

    Box2d bounding_box = Node3d::GetBoundingBox(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i]);

    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const common::math::LineSegment2d& linesegment :
           obstacle_linesegments) {
        if (bounding_box.HasOverlap(linesegment)) {
          ADEBUG("collision start at x: " << linesegment.start().x());
          ADEBUG("collision start at y: " << linesegment.start().y());
          ADEBUG("collision end at x: " << linesegment.end().x());
          ADEBUG("collision end at y: " << linesegment.end().y());
          return false;
        }
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedsSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  end_node->SetPre(current_node);
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;
  if (next_node_index < static_cast<double>(next_node_num_) / 2) {
    -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(next_node_index);
    traveled_distance = step_size_;
  } else {
    size_t index = next_node_index - next_node_num_ / 2;
    (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
        static_cast<double>(index);
    traveled_distance = -step_size_;
  }
  // tacke above motion primitive to generate a curve driving the car to a
  // different grid
  double arc = M_SQRT2 * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  for (size_t i = 0; i < arc / step_size_; i++) {
    const double next_x = last_x + traveled_distance * cos(last_phi);
    const double next_y = last_y + traveled_distance * sin(last_phi);
    const double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base() * std::tan(steering));
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }

  // check if the vehicle runs out of XY boundary
  if (intermediate_x.back() < XYbounds_[0] ||
      intermediate_x.back() > XYbounds_[1] ||
      intermediate_y.back() < XYbounds_[2] ||
      intermediate_y.back() > XYbounds_[3]) {
    return nullptr;
  }

  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 planner_open_space_config_));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}

double HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                      std::shared_ptr<Node3d> next_node) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));

  // evaluate heuritstic cost
  double optimal_path_cost = 0.0;
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_backward_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                      next_node->GetY());
}

bool HybridAStar::GetResult(HybridAStarResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      AERROR("result sizes check failed");
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      AERROR("states sizes are not equal");
      return false;
    }
    std::reverse(std::begin(x), std::end(x));
    std::reverse(std::begin(y), std::end(y));
    std::reverse(std::begin(phi), std::end(phi));
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(std::end(hybrid_a_x), std::begin(x), std::end(x));
    hybrid_a_y.insert(std::end(hybrid_a_y), std::begin(y), std::end(y));
    hybrid_a_phi.insert(std::end(hybrid_a_phi), std::begin(phi), std::end(phi));
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  std::reverse(std::begin(hybrid_a_x), std::end(hybrid_a_x));
  std::reverse(std::begin(hybrid_a_y), std::end(hybrid_a_y));
  std::reverse(std::begin(hybrid_a_phi), std::end(hybrid_a_phi));
  result->x = std::move(hybrid_a_x);
  result->y = std::move(hybrid_a_y);
  result->phi = std::move(hybrid_a_phi);

  if (!GetTemporalProfile(result)) {
    AERROR("Generate speed profile failed");
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->v.size() ||
      result->x.size() != result->phi.size()) {
    AERROR("state sizes are equal."
           << " result->x.size(): " << result->x.size() << " result->y.size(): "
           << result->y.size() << " result->v.size(): " << result->v.size()
           << " result->phi.size(): " << result->phi.size());
    return false;
  }

  if (result->a.size() != result->steer.size() ||
      result->x.size() != (result->a.size() - 1)) {
    AERROR("control sizes are neither equal nor right");
    AERROR("acceleration size: " << result->a.size());
    AERROR("steer size: " << result->steer.size());
    AERROR("x size: " << result->x.size());
    return false;
  }
  return true;
}

bool HybridAStar::GetTemporalProfile(HybridAStarResult* result) {
  std::vector<HybridAStarResult> partitioned_result;
  if (!TrajectoryPartition(*result, &partitioned_result)) {
    AERROR("Trajectory partition failed");
    return false;
  }
}

bool HybridAStar::TrajectoryPartition(
    const HybridAStarResult& result,
    std::vector<HybridAStarResult>* partitioned_result) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;

  if (x.size() != y.size() || x.size() != phi.size()) {
    AERROR(
        "states sizes are not equal when do trajectory partitioning of Hybrid "
        "A Star result");
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto* current_traj = &(partitioned_result->back());
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.Angle();
  bool current_gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
      M_PI_2;
  for (size_t i = 0; i < horizon - 1; i++) {
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
        M_PI_2;
    if (gear != current_gear) {
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());

  const auto start_timestamp = std::chrono::system_clock::now();

  // Retrieve v, a, and steer from path
  for (auto& result : *partitioned_result) {
    if (FLAGS_use_s_curve_speed_smooth) {
      if (!GenerateSCurveSpeedAcceleration(&result)) {
        AERROR("GenerateSCurveSpeedAcceleration failed");
        return false;
      }
    } else {
      if (!GenerateSpeedAcceleration(&result)) {
        AERROR("GenerateSpeedAcceleration failed");
        return false;
      }
    }
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_timestamp - start_timestamp)
                        .count();
  ADEBUG("speed profile total time: " << diff << " ms.");
  return true;
}

bool HybridAStar::GenerateSpeedAcceleration(HybridAStarResult* result) {
  // Sanity check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR("result size check when generating speed and acceleration failed");
    return false;
  }

  const size_t horizon = result->x.size();

  // load velocity from position
  // initial and end speed are set to be zero
  result->v.resize(horizon);

  result->v[0] = 0.0;
  for (size_t i = 1; i < horizon - 1; i++) {
    result->v[i] = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                        std::cos(result->phi[i]) +
                    ((result->x[i] - result->x[i - 1]) / delta_t_) *
                        std::cos(result->phi[i])) /
                       2.0 +
                   (((result->y[i + 1] - result->y[i]) / delta_t_) *
                        std::sin(result->phi[i]) +
                    ((result->y[i] - result->y[i - 1]) / delta_t_) *
                        std::sin(result->phi[i])) /
                       2.0;
  }
  result->v[horizon - 1] = 0.0;

  // populate acceleration from velocity
  result->a.resize(horizon - 1);
  for (size_t i = 0; i < horizon - 1; i++) {
    result->a[i] = (result->v[i + 1] - result->v[i]) / delta_t_;
  }

  // populate steering from phi
  result->steer.resize(horizon - 1);
  for (size_t i = 0; i < horizon - 1; i++) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            vehicle_param_.wheel_base() / step_size_;
    if (result->v[i] > 0.0) {
      discrete_steer = std::atan(discrete_steer);
    } else {
      discrete_steer = std::atan(-discrete_steer);
    }
    result->steer[i] = discrete_steer;
  }
  return true;
}

bool HybridAStar::GenerateSCurveSpeedAcceleration(HybridAStarResult* result) {
  CHECK_NOTNULL(result);
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR("result size check when generating speed and acceleration failed");
    return false;
  }
  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    AERROR("result sizes not equal");
    return false;
  }

  double init_heading = result->phi.front();
  const Vec2d init_tracking_vec(result->x[1] - result->x[0],
                                result->y[1] - result->y[0]);
  const bool gear = std::abs(common::math::NormalizeAngle(
                        init_heading - init_tracking_vec.Angle())) < M_PI_2;

  // number of points in path
  size_t path_points_size = result->x.size();

  result->accumulated_s.resize(path_points_size);

  double accumulated_s = 0.0;
  auto last_x = result->x.front();
  auto last_y = result->y.front();

  for (size_t i = 0; i < path_points_size; i++) {
    double x_diff = result->x[i] - last_x;
    double y_diff = result->y[i] - last_y;
    accumulated_s += std::hypot(x_diff, y_diff);

    result->accumulated_s[i] = accumulated_s;
    last_x = result->x[i];
    last_y = result->y[i];
  }

  // assume static init state
  const double init_v = 0.0;
  const double init_a = 0.0;

  const auto& opt_conf =
      planner_open_space_config_.warm_start_config().s_curve_config();

  // minimum time speed optimization
  const double max_forward_v = opt_conf.max_forward_velocity();
  const double max_reverse_v = opt_conf.max_reverse_velocity();
  const double max_forward_a = opt_conf.max_forward_acceleration();
  const double max_reverse_a = opt_conf.max_reverse_acceleration();
  const double max_acc_jerk = opt_conf.max_acceleration_jerk();

  const double dt = planner_open_space_config_.warm_start_config().delta_t();
  const double looseness_ratio =
      planner_open_space_config_.warm_start_config().time_looseness_ratio();
  const double max_path_time =
      planner_open_space_config_.warm_start_config().max_path_time();

  SpeedData speed_data;

  //
  const double path_length = result->accumulated_s.back();

  const double total_t = std::max(
      gear ? looseness_ratio *
                 (max_forward_v * max_forward_v + path_length * max_forward_a) /
                 (max_forward_a * max_forward_v)
           : looseness_ratio *
                 (max_reverse_v * max_reverse_v + path_length * max_reverse_a) /
                 (max_reverse_a * max_reverse_v),
      max_path_time);

  const size_t num_of_knots = static_cast<size_t>(total_t / dt) + 1;

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(
      num_of_knots, dt, {0.0, std::abs(init_v), std::abs(init_a)});

  // set constraints
  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, path_length});

  const double max_v = gear ? max_forward_v : max_reverse_v;
  const double max_acc = gear ? max_forward_a : max_reverse_a;

  const auto upper_dx = std::max(max_v, std::abs(init_v));
  const auto upper_ddx = std::max(max_acc, std::abs(init_a));
  std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                   {0.0, upper_dx});
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {-upper_ddx, upper_ddx});

  // set end constraints
  x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
  dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
  ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

  // Sf
  std::vector<double> x_ref(num_of_knots, path_length);

  const double w_x_ref = opt_conf.weight_x_ref();
  const double w_ddx = opt_conf.weight_ddx();
  const double w_dddx = opt_conf.weight_dddx();

  // Setup piecewise optimization problem
  piecewise_jerk_problem.set_x_ref(w_x_ref, std::move(x_ref));
  piecewise_jerk_problem.set_weight_ddx(w_ddx);
  piecewise_jerk_problem.set_weight_dddx(w_dddx);
  piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

  // solve
  if (!piecewise_jerk_problem.Optimize()) {
    AERROR("Piecewise jerk speed optimizer failed");
    return false;
  }

  // extract output
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

  speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  constexpr double kEpsilon = 1e-6;
  constexpr double sEpsilon = 1e-6;

  for (size_t i = 1; i < num_of_knots; i++) {
    // s should be monotonically increasing along the path
    if (s[i - 1] - s[i] > kEpsilon) {
      ADEBUG("unexpected decreasing s in speed smoothing at time "
             << static_cast<double>(i) * dt << " with total time " << total_t);
      break;
    }
    speed_data.AppendSpeedPoint(s[i], dt * static_cast<double>(i), ds[i],
                                dds[i], (dds[i] - dds[i - 1]) / dt);
    // cut the speed data when it is about to meet the end condition
    if (path_length - s[i] < sEpsilon) {
      break;
    }
  }

  // combine speed and path profile
  DiscretizedPath path_data;
  for (size_t i = 0; i < path_points_size; i++) {
    common::PathPoint path_point;
    path_point.set_x(result->x[i]);
    path_point.set_y(result->y[i]);
    path_point.set_theta(result->phi[i]);
    path_point.set_s(result->accumulated_s[i]);
    path_data.push_back(std::move(path_point));
  }

  HybridAStarResult combined_result;

  const double kDenseTimeResolution =
      planner_open_space_config_.warm_start_config().dense_time_resolution();

  // dont know what it means.
  const double time_horizon =
      speed_data.TotalTime() + kDenseTimeResolution * 1e-6;

  if (path_data.empty()) {
    AERROR("path data is empty");
    return false;
  }

  for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
       cur_rel_time += kDenseTimeResolution) {
    common::SpeedPoint speed_point;
    if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR("Fail to get speed point with relative time " << cur_rel_time);
      return false;
    }

    if (speed_point.s() > path_data.Length()) {
      break;
    }

    common::PathPoint path_point = path_data.Evaluate(speed_point.s());

    combined_result.x.push_back(path_point.x());
    combined_result.y.push_back(path_point.y());
    combined_result.phi.push_back(path_point.theta());
    combined_result.accumulated_s.push_back(path_point.s());

    if (!gear) {
      combined_result.v.push_back(-speed_point.v());
      combined_result.a.push_back(-speed_point.a());
    } else {
      combined_result.v.push_back(speed_point.v());
      combined_result.a.push_back(speed_point.a());
    }
  }

  combined_result.a.pop_back();

  // update size
  path_points_size = combined_result.x.size();

  // update steer
  combined_result.steer.resize(path_points_size);
  for (size_t i = 0; i < path_points_size - 1; i++) {
    double discrete_steer =
        (combined_result.phi[i + 1] - combined_result.phi[i]) *
        vehicle_param_.wheel_base() /
        (combined_result.accumulated_s[i + 1] -
         combined_result.accumulated_s[i]);
    combined_result.steer[i] =
        gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
  }

  *result = combined_result;
  return true;
}

bool HybridAStar::Plan(
    const double sx, const double sy, const double sphi, const double ex,
    const double ey, const double ephi, const std::vector<double>& XYbounds,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec,
    HybridAStarResult* result) {
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<common::math::LineSegment2d> obstacle_linesegments;
    for (size_t i = 0; i < vertices_num - 1; i++) {
      common::math::LineSegment2d line_segment = common::math::LineSegment2d(
          obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

  // load XYbounds;
  XYbounds_ = XYbounds;
  start_node_.reset(
      new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
  end_node_.reset(
      new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));
  if (!ValidityCheck(start_node_)) {
    AERROR("start node in collision with obstacles");
    return false;
  }

  if (!ValidityCheck(end_node_)) {
    AERROR("end node in collision with obstacles");
    return false;
  }
  const auto map_start_time = std::chrono::system_clock::now();
  grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_,
                                                  obstacles_linesegments_vec_);
  const auto map_end_time = std::chrono::system_clock::now();
  const auto map_time_cost =
      std::chrono::duration_cast<std::chrono::milliseconds>(map_end_time -
                                                            map_start_time)
          .count();
  ADEBUG("map time " << map_time_cost << " ms");
  // load open set, pq
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

  // Hybrid A*
  size_t explored_node_num = 0;
  const auto astar_start_time = std::chrono::system_clock::now();
  int heuristic_time = 0;
  int rs_time = 0;
  while (!open_pq_.empty()) {
    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    // check if analytic curve could connect the current configuration to the
    // end configuration without collision. if so, search ends.
    const auto rs_start_time = std::chrono::system_clock::now();
    if (AnalyticExpansion(current_node)) {
      break;
    }
    const auto rs_end_time = std::chrono::system_clock::now();
    rs_time += std::chrono::duration_cast<std::chrono::milliseconds>(
                   rs_end_time - rs_start_time)
                   .count();
    close_set_.emplace(current_node->GetIndex(), current_node);
    for (size_t i = 0; i < next_node_num_; i++) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // check if the node is in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      // collision check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        explored_node_num++;
        const auto start_time = std::chrono::system_clock::now();
        CalculateNodeCost(current_node, next_node);
        const auto end_time = std::chrono::system_clock::now();
        heuristic_time += std::chrono::duration_cast<std::chrono::milliseconds>(
                              end_time - start_time)
                              .count();
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
  }
  if (final_node_ == nullptr) {
    ADEBUG("Hybrid A searching return null ptr (open_set ran out)");
    return false;
  }

  if (!GetResult(result)) {
    ADEBUG("GetResult failed");
    return false;
  }
  const auto astar_end_time = std::chrono::system_clock::now();
  ADEBUG("explored node num is " << explored_node_num);
  ADEBUG("heuristic time is " << heuristic_time);
  ADEBUG("reed shepp time is " << rs_time);
  ADEBUG("hybrid astar total time is "
         << std::chrono::duration_cast<std::chrono::milliseconds>(
                astar_end_time - astar_start_time)
                .count());
}

}  // namespace planning
}  // namespace autoagric