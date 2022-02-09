#include "planning/open_space/trajectory_smoother/iterative_anchoring_smoother.h"

#include <chrono>
#include <limits>

#include "autoagric/common/pnc_point.pb.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/macro.h"
#include "common/math/box2d.h"
#include "common/math/math_utils.h"
#include "planning/math/discrete_points_math.h"
#include "planning/math/discretized_points_smoothing/cos_theta_smoother.h"
#include "planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace autoagric {
namespace planning {

using common::PathPoint;
using common::math::Box2d;
using common::math::LineSegment2d;
using common::math::NormalizeAngle;
using common::math::Vec2d;

IterativeAnchoringSmoother::IterativeAnchoringSmoother(
    const PlannerOpenSpaceConfig& planner_open_space_config) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  ego_length_ = vehicle_param.length();
  ego_width_ = vehicle_param.width();
  center_shift_distance_ =
      ego_length_ / 2.0 - vehicle_param.back_edge_to_center();
  planner_open_space_config_ = planner_open_space_config;
}

bool IterativeAnchoringSmoother::Smooth(
    const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    DiscretizedTrajectory* discretized_trajectory) {
  if (xWS.cols() < 2) {
    AERROR("reference points size smaller than two, smoother early returned");
    return false;
  }
  const auto start_timestamp = std::chrono::system_clock::now();

  // set gear of the trajectory
  gear_ = CheckGear(xWS);

  // set obstacle in form of linesegments
  std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    std::vector<LineSegment2d> obstacle_linesegments;
    size_t vertices_num = obstacle_vertices.size();
    for (size_t i = 0; i < vertices_num - 1; i++) {
      obstacle_linesegments.emplace_back(obstacle_vertices[i],
                                         obstacle_vertices[i + 1]);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

  // interpolate trajectory
  DiscretizedPath warm_start_path;
  const size_t xWS_size = xWS.cols();
  double accumulated_s = 0.0;
  Vec2d last_path_point(xWS(0, 0), xWS(1, 0));
  for (size_t i = 0; i < xWS_size; i++) {
    Vec2d cur_path_point(xWS(0, i), xWS(1, i));
    accumulated_s += cur_path_point.DistanceTo(last_path_point);
    PathPoint path_point;
    path_point.set_x(xWS(0, i));
    path_point.set_y(xWS(1, i));
    path_point.set_theta(xWS(2, i));
    path_point.set_s(accumulated_s);
    warm_start_path.emplace_back(std::move(path_point));
    last_path_point = cur_path_point;
  }

  const double interpolated_delta_s =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .interpolated_delta_s();
  std::vector<std::pair<double, double>> interpolated_warm_start_point2ds;
  double path_length = warm_start_path.Length();
  double delta_s = path_length / std::ceil(path_length / interpolated_delta_s);
  path_length += delta_s * 1e-6;
  for (double s = 0.0; s < path_length; s += delta_s) {
    const auto point2d = warm_start_path.Evaluate(s);
    interpolated_warm_start_point2ds.emplace_back(point2d.x(), point2d.y());
  }

  const size_t interpolated_size = interpolated_warm_start_point2ds.size();
  if (interpolated_size < 4) {
    AERROR(
        "interpolated_warm_start_path smaller than 4, can't enforce heading "
        "continuity");
    return false;
  } else if (interpolated_size < 6) {
    ADEBUG(
        "interpolated_warm_start_path smaller than 6, can't enforce initial "
        "zero kappa");
    enforce_initial_kappa_ = false;
  } else {
    enforce_initial_kappa_ = true;
  }

  // adjust heading to ensure heading continuity
  AdjustStartEndHeading(xWS, &interpolated_warm_start_point2ds);

  // Reset path profile by discrete point heading and curvature estimation
  DiscretizedPath interpolated_warm_start_path;
  if (!SetPathProfile(interpolated_warm_start_point2ds,
                      &interpolated_warm_start_path)) {
    AERROR("Set path profle failed");
    return false;
  }

  // generate feasible bounds for each path point
  std::vector<double> bounds;
  if (!GenerateInitialBounds(interpolated_warm_start_path, &bounds)) {
    AERROR(
        "Generate initial bounds failed. Potential collision between obstacles "
        "and warm start path");
    return false;
  }

  // check initial path collision avoidance.
  input_colliding_point_index_.clear();
  if (!CheckCollisionAvoidance(interpolated_warm_start_path,
                               &input_colliding_point_index_)) {
    AERROR("Interpolated input path points colliding with obstacle");
  }

  const auto path_smooth_start_timestamp = std::chrono::system_clock::now();
  // smooth path
  DiscretizedPath smoothed_path_points;
  if (!SmoothPath(interpolated_warm_start_path, bounds,
                  &smoothed_path_points)) {
    return false;
  }

  const auto path_smooth_end_timestamp = std::chrono::system_clock::now();
  const auto path_smooth_diff =
      std::chrono::duration<double, std::milli>(path_smooth_end_timestamp -
                                                path_smooth_start_timestamp)
          .count();
  ADEBUG("iterative anchoring path smoother time: " << path_smooth_diff
                                                    << " ms");

  const auto speed_smooth_start_timestamp = std::chrono::system_clock::now();

  SpeedData smoothed_speeds;
  if (!SmoothSpeed(init_a, init_v, smoothed_path_points.Length(),
                   &smoothed_speeds)) {
    return false;
  }

  const auto speed_smooth_end_timestamp = std::chrono::system_clock::now();
  const auto speed_smooth_diff =
      std::chrono::duration<double, std::milli>(speed_smooth_end_timestamp -
                                                speed_smooth_start_timestamp)
          .count();
  ADEBUG("iterative anchoring speed smoother time: " << speed_smooth_diff
                                                     << " ms");

  if (!CombinePathAndSpeed(smoothed_path_points, smoothed_speeds,
                           discretized_trajectory)) {
    return false;
  }

  AdjustPathAndSpeedByGear(discretized_trajectory);

  const auto end_timestamp = std::chrono::system_clock::now();
  const auto diff =
      std::chrono::duration<double, std::milli>(end_timestamp - start_timestamp)
          .count();
  ADEBUG("iterative anchoring smoother total time: " << diff << " ms");
  ADEBUG("discretized_trajectory size: " << discretized_trajectory->size());
  return true;
}

bool IterativeAnchoringSmoother::CheckGear(const Eigen::MatrixXd& xWS) {
  CHECK_GT(xWS.size(), 1);
  double init_heading_angle = xWS(2, 0);
  const Vec2d init_tracking_vector(xWS(0, 1) - xWS(0, 0),
                                   xWS(1, 1) - xWS(1, 0));
  double init_tracking_angle = init_tracking_vector.Angle();
  return std::abs(NormalizeAngle(init_tracking_angle - init_heading_angle)) <
         M_PI_2;
}

void IterativeAnchoringSmoother::AdjustStartEndHeading(
    const Eigen::MatrixXd& xWS,
    std::vector<std::pair<double, double>>* point2d) {
  // sanity check
  CHECK_NOTNULL(point2d);
  CHECK_GT(xWS.cols(), 1);
  CHECK_GT(point2d->size(), 3U);

  /**
   * @note hard-codingly change the positions of the second and the second last
   * points according to the first and last points' heading angle.
   * @todo (chufan) refactor after stablized
   */

  // set initial heading and bounds
  const double initial_heading = xWS(2, 0);
  const double end_heading = xWS(2, xWS.cols() - 1);

  const double first_to_second_dx = point2d->at(1).first - point2d->at(0).first;
  const double first_to_second_dy =
      point2d->at(1).second - point2d->at(0).second;
  const double first_to_second_s =
      std::hypot(first_to_second_dx, first_to_second_dy);
  Vec2d first_point(point2d->at(0).first, point2d->at(0).second);
  Vec2d initial_vec(first_to_second_s, 0.0);
  initial_vec.SelfRotate(gear_ ? initial_heading
                               : NormalizeAngle(initial_heading + M_PI));
  initial_vec += first_point;
  point2d->at(1) = std::make_pair(initial_vec.x(), initial_vec.y());

  const size_t path_size = point2d->size();
  const double second_last_to_last_dx =
      point2d->at(path_size - 1).first - point2d->at(path_size - 2).first;
  const double second_last_to_last_dy =
      point2d->at(path_size - 1).second - point2d->at(path_size - 2).second;
  const double second_last_to_last_s =
      std::hypot(second_last_to_last_dx, second_last_to_last_dy);
  Vec2d last_point(point2d->at(path_size - 1).first,
                   point2d->at(path_size - 1).second);
  Vec2d end_vec(second_last_to_last_s, 0);
  end_vec.SelfRotate(gear_ ? NormalizeAngle(end_heading + M_PI) : end_heading);
  end_vec += last_point;
  point2d->at(path_size - 2) = std::make_pair(end_vec.x(), end_vec.y());
}

bool IterativeAnchoringSmoother::SetPathProfile(
    const std::vector<std::pair<double, double>>& point2d,
    DiscretizedPath* raw_path_points) {
  CHECK_NOTNULL(raw_path_points);
  raw_path_points->clear();
  // compute path profile
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!DiscretePointsMath::ComputePathPofile(point2d, &headings, &accumulated_s,
                                             &kappas, &dkappas)) {
    return false;
  }

  CHECK_EQ(point2d.size(), headings.size());
  CHECK_EQ(point2d.size(), kappas.size());
  CHECK_EQ(point2d.size(), dkappas.size());
  CHECK_EQ(point2d.size(), accumulated_s.size());

  // load into path point
  size_t points_size = point2d.size();
  for (size_t i = 0; i < points_size; i++) {
    PathPoint path_point;
    path_point.set_x(point2d[i].first);
    path_point.set_y(point2d[i].second);
    path_point.set_theta(headings[i]);
    path_point.set_kappa(kappas[i]);
    path_point.set_dkappa(dkappas[i]);
    path_point.set_s(accumulated_s[i]);
    raw_path_points->emplace_back(std::move(path_point));
  }
  return true;
}

bool IterativeAnchoringSmoother::GenerateInitialBounds(
    const DiscretizedPath& path_points, std::vector<double>* initial_bounds) {
  CHECK_NOTNULL(initial_bounds);
  initial_bounds->clear();

  const bool estimate_bound =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .estimate_bound();
  const double default_bound =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .default_bound();
  const double vehicle_shortest_dimension =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .vehicle_shortest_dimension();
  const double kEpsilon = 1e-8;

  if (!estimate_bound) {
    std::vector<double> default_bounds(path_points.size(), default_bound);
    *initial_bounds = std::move(default_bounds);
    return true;
  }
  const auto start_timestamp = std::chrono::system_clock::now();
  for (const auto& path_point : path_points) {
    double min_bound = std::numeric_limits<double>::infinity();
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const auto& linesegment : obstacle_linesegments) {
        min_bound =
            std::min(min_bound,
                     linesegment.DistanceTo({path_point.x(), path_point.y()}));
      }
    }
    min_bound -= vehicle_shortest_dimension;
    min_bound = min_bound < kEpsilon ? 0.0 : min_bound;
    initial_bounds->push_back(min_bound);
  }
  const auto end_timestamp = std::chrono::system_clock::now();
  const auto diff =
      std::chrono::duration<double, std::milli>(end_timestamp - start_timestamp)
          .count();
  ADEBUG("Time consumption of calculating minimum bound: " << diff << " ms");

  return true;
}

bool IterativeAnchoringSmoother::CheckCollisionAvoidance(
    const DiscretizedPath& path_points,
    std::vector<size_t>* colliding_point_index) {
  CHECK_NOTNULL(colliding_point_index);

  colliding_point_index->clear();
  size_t path_points_size = path_points.size();
  for (size_t i = 0; i < path_points_size; i++) {
    // avoid duplicated collision check
    bool skip_checking = false;
    for (const auto index : input_colliding_point_index_) {
      if (i == index) {
        skip_checking = true;
        break;
      }
    }

    if (skip_checking) {
      continue;
    }

    const double heading = gear_
                               ? path_points[i].theta()
                               : NormalizeAngle(path_points[i].theta() + M_PI);

    Box2d ego_box(
        {path_points[i].x() + center_shift_distance_ * std::cos(heading),
         path_points[i].y() + center_shift_distance_ * std::sin(heading)},
        heading, ego_length_, ego_width_);

    bool is_colliding = false;

    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const auto& linesegment : obstacle_linesegments) {
        /**
         * @todo move to conf
         */
        if (ego_box.DistanceTo(linesegment) < 0.1) {
          colliding_point_index->push_back(i);
          ADEBUG("point at " << i << " collided with LineSegment "
                             << linesegment.DebugString());
          is_colliding = true;
          break;
        }
      }
      if (is_colliding) {
        break;
      }
    }
  }

  if (!colliding_point_index->empty()) {
    return false;
  }
  return true;
}

bool IterativeAnchoringSmoother::SmoothPath(
    const DiscretizedPath& path_points, const std::vector<double>& bounds,
    DiscretizedPath* smoothed_path_points) {
  /**
   * @todo refactor using FemPosDeviationSmoother
   */

  // sanity check
  CHECK_EQ(path_points.size(), bounds.size());

  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<double> flexible_bounds;

  for (const auto& path_point : path_points) {
    raw_point2d.emplace_back(path_point.x(), path_point.y());
  }
  flexible_bounds = bounds;

  CosThetaSmoother cos_theta_smoother(
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .cos_theta_smoother_config());

  const size_t max_iteration_num =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .max_iteration_num();

  bool is_collision_free = false;
  std::vector<size_t> colliding_point_index;
  std::vector<std::pair<double, double>> smoothed_point2d;
  size_t counter = 0;

  while (!is_collision_free) {
    if (counter > max_iteration_num) {
      AERROR("Maximum iteration reached, path smoother early stops");
      return true;
    }

    AdjustPathBounds(colliding_point_index, &flexible_bounds);

    std::vector<double> opt_x;
    std::vector<double> opt_y;
    if (!cos_theta_smoother.Solve(raw_point2d, flexible_bounds, &opt_x,
                                  &opt_y)) {
      AERROR("Smoothing path failed");
      return false;
    }

    if (opt_x.size() < 2 || opt_y.size() < 2) {
      AERROR("Result of cos_theta_smoother is wrong. Size less than 2");
      return false;
    }

    CHECK_EQ(opt_x.size(), opt_y.size());

    size_t point_size = opt_x.size();
    smoothed_point2d.resize(point_size);
    for (size_t i = 0; i < point_size; i++) {
      smoothed_point2d[i] = std::make_pair(opt_x[i], opt_y[i]);
    }

    if (!SetPathProfile(smoothed_point2d, smoothed_path_points)) {
      AERROR("Set path profile failed");
      return false;
    }

    is_collision_free =
        CheckCollisionAvoidance(*smoothed_path_points, &colliding_point_index);

    ADEBUG("Loop iteration num is " << counter);
    counter++;
  }
  return true;
}

bool IterativeAnchoringSmoother::SmoothSpeed(const double init_a,
                                             const double init_v,
                                             const double path_length,
                                             SpeedData* smoothed_speeds) {
  CHECK_NOTNULL(smoothed_speeds);

  const auto& smoother_config =
      planner_open_space_config_.iterative_anchoring_smoother_config();
  const double max_forward_v = smoother_config.max_forward_v();
  const double max_reverse_v = smoother_config.max_reverse_v();
  const double max_forward_a = smoother_config.max_forward_acc();
  const double max_reverse_a = smoother_config.max_reverse_acc();
  const double max_acc_jerk = smoother_config.max_acc_jerk();
  const double dt = smoother_config.delta_t();
  const double loosen_ratio = smoother_config.time_slack_ratio();

  const double total_time =
      gear_
          ? loosen_ratio *
                (max_forward_v * max_forward_v + path_length * max_forward_a) /
                (max_forward_v * max_forward_a)
          : loosen_ratio *
                (max_reverse_v * max_reverse_v + path_length * max_reverse_a) /
                (max_reverse_a * max_reverse_v);
  ADEBUG("total time is " << total_time);

  const size_t num_of_knots = static_cast<size_t>(total_time / dt) + 1;

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(
      num_of_knots, dt, {0.0, std::abs(init_v), std::abs(init_a)});

  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, path_length});

  const double max_v = gear_ ? max_forward_v : max_reverse_v;
  const double max_a = gear_ ? max_forward_a : max_reverse_a;

  const auto upper_dx = std::fmax(max_v, std::abs(init_v));
  std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                   {0.0, upper_dx});
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {-max_a, max_a});

  x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
  dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
  // ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

  std::vector<double> x_ref(num_of_knots, path_length);

  const double weight_x_ref = smoother_config.s_curve_config().ref_s_weight();
  const double weight_dx = smoother_config.s_curve_config().acc_weight();
  const double weight_ddx = smoother_config.s_curve_config().jerk_weight();

  // ADEBUG("weight_x_ref: " << weight_x_ref);
  // ADEBUG("weight_dx: " << weight_dx);
  // ADEBUG("weight_ddx: " << weight_ddx);

  // ADEBUG("upper_dx: " << upper_dx);
  // ADEBUG("max_a: " << max_a);
  // ADEBUG("max_acc_jerk: " << max_acc_jerk);

  // ADEBUG("path_length: " << path_length);
  // ADEBUG("total_time: " << total_time);
  // ADEBUG("dt: " << dt);
  // ADEBUG("num_of_knots: " << num_of_knots);

  piecewise_jerk_problem.set_x_ref(weight_x_ref, std::move(x_ref));
  piecewise_jerk_problem.set_weight_ddx(weight_dx);
  piecewise_jerk_problem.set_weight_dddx(weight_ddx);
  piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

  const int max_num_of_iterations =
      smoother_config.s_curve_config().max_num_of_iterations();

  // solve
  if (!piecewise_jerk_problem.Optimize(max_num_of_iterations)) {
    AERROR("Piecewise jerk speed optimizer failed");

    for (size_t i = 0; i < piecewise_jerk_problem.debug_x().size(); i++) {
      ADEBUG(std::setprecision(3)
             << piecewise_jerk_problem.debug_x()[i] << ", "
             << piecewise_jerk_problem.debug_dx()[i] << ", "
             << piecewise_jerk_problem.debug_ddx()[i]);
    }

    return false;
  }

  // Extract output
  const auto& s = piecewise_jerk_problem.opt_x();
  const auto& ds = piecewise_jerk_problem.opt_dx();
  const auto& dds = piecewise_jerk_problem.opt_ddx();

  smoothed_speeds->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  constexpr double sEpsilon = 1e-6;

  for (size_t i = 1; i < num_of_knots; i++) {
    if (s[i - 1] - s[i] > sEpsilon) {
      AWARN("unexpected decreasing s in speed acceleration at time "
            << static_cast<double>(i) * dt << " with total time "
            << total_time);
      // ADEBUG("s: " << s[i - 1] << "\nt: " << static_cast<double>(i - 1) * dt
      //              << "\nv: " << ds[i - 1] << "\na: " << dds[i - 1]);
      // ADEBUG("s: " << s[i] << "\nt: " << static_cast<double>(i) * dt
      //              << "\nv: " << ds[i] << "\na: " << dds[i]);
      break;
    }
    smoothed_speeds->AppendSpeedPoint(s[i], dt * static_cast<double>(i), ds[i],
                                      dds[i], (dds[i] - dds[i - 1]) / dt);
    if (path_length - s[i] < sEpsilon) {
      break;
    }
  }
  return true;
}

void IterativeAnchoringSmoother::AdjustPathBounds(
    const std::vector<size_t>& colliding_point_index,
    std::vector<double>* bounds) {
  CHECK_NOTNULL(bounds);
  CHECK_GT(bounds->size(), 2U);

  const double collision_decrease_ratio =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .collision_decrease_ratio();

  for (const auto index : colliding_point_index) {
    bounds->at(index) *= collision_decrease_ratio;
  }

  // anchor the end point to enforce the initial and end headiing continuity
  // and zero kappa
  bounds->at(0) = 0.0;
  bounds->at(1) = 0.0;
  bounds->at(bounds->size() - 1) = 0.0;
  bounds->at(bounds->size() - 2) = 0.0;
  if (enforce_initial_kappa_) {
    bounds->at(2) = 0.0;
  }
}

bool IterativeAnchoringSmoother::CombinePathAndSpeed(
    const DiscretizedPath& path_points, const SpeedData& speed_points,
    DiscretizedTrajectory* discretized_trajectory) {
  CHECK_NOTNULL(discretized_trajectory);
  discretized_trajectory->clear();

  const double kDenseTimeResolution =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .dense_time_resolution();
  const double time_horizion =
      speed_points.TotalTime() + kDenseTimeResolution * 1e-6;
  if (path_points.empty()) {
    AERROR("path data is empty");
    return false;
  }

  ADEBUG("speed_points.TotalTime() " << speed_points.TotalTime());
  for (double cur_rel_time = 0.0; cur_rel_time < time_horizion;
       cur_rel_time += kDenseTimeResolution) {
    common::SpeedPoint speed_point;
    if (!speed_points.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR("Fail to get speed point with relative time " << cur_rel_time);
      return false;
    }

    if (speed_point.s() > path_points.Length()) {
      break;
    }
    common::PathPoint path_point = path_points.Evaluate(speed_point.s());

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->Swap(&path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t());
    discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  ADEBUG("path length before combine " << path_points.Length());
  ADEBUG("trajectory length after combine "
         << discretized_trajectory->GetSpatialLength());
  return true;
}

void IterativeAnchoringSmoother::AdjustPathAndSpeedByGear(
    DiscretizedTrajectory* discretized_trajectory) {
  if (gear_) {
    return;
  }

  CHECK_NOTNULL(discretized_trajectory);

  std::for_each(
      discretized_trajectory->begin(), discretized_trajectory->end(),
      [](common::TrajectoryPoint& trajectory_point) {
        trajectory_point.mutable_path_point()->set_theta(
            NormalizeAngle(trajectory_point.path_point().theta() + M_PI));
        trajectory_point.mutable_path_point()->set_s(
            -1.0 * trajectory_point.path_point().s());
        trajectory_point.mutable_path_point()->set_kappa(
            -1.0 * trajectory_point.path_point().kappa());
        trajectory_point.set_v(-1.0 * trajectory_point.v());
        trajectory_point.set_a(-1.0 * trajectory_point.a());
      });
}

}  // namespace planning
}  // namespace autoagric