#include "planning/static/static_path_generator.h"

#include <Eigen/Core>
#include <chrono>

#include "autoagric/common/pnc_point.pb.h"
#include "common/macro.h"
#include "common/math/math_utils.h"
#include "common/math/matrix_operations.h"
#include "common/math/vec2d.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/speed/speed_data.h"
#include "planning/common/speed_profile_generator.h"
#include "planning/math/discrete_points_math.h"
#include "planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace autoagric {
namespace planning {

namespace {
constexpr double kDoubleEpsilon = 1e-3;
}

using common::TrajectoryPoint;
using common::math::Vec2d;
using common::util::CSVReader;
using common::util::StaticPathResult;

StaticPathGenerator::StaticPathGenerator(const std::string& file_path)
    : file_path_(file_path) {
  AINFO("Load static path from " << file_path);
}

bool StaticPathGenerator::Init(const StaticPathConfig& config) {
  config_ = config;

  csv_reader_.reset(new CSVReader());

  csv_reader_->LoadDataFromFile(file_path_.c_str(), &path_);

  if (path_.x.size() != path_.y.size() || path_.x.size() != path_.phi.size()) {
    AERROR("sizes of necessary states of overall path are not equal");
    return false;
  }

  path_length_ = path_.x.size();

  current_start_index_ = 0;

  const auto& cos_theta_config =
      config_.smoother_conf().discrete_points().cos_theta_smoothing();

  ADEBUG(cos_theta_config.DebugString());

  cos_theta_smoother_.reset(new CosThetaSmoother(cos_theta_config));

  return true;
}

bool StaticPathGenerator::Proc() {
  if (!GetTemporalProfile(&path_)) {
    AERROR("Generating static path failed");
    return false;
  }

  AINFO("Generating static path successfully");
  return true;
}

bool StaticPathGenerator::GenerateSmoothPathProfle(StaticPathResult* result) {
  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    AERROR("sizes of necessary states of partitioned path are not equal");
    return false;
  }

  if (result->x.size() < 3) {
    AWARN("Less than 3 points. No need to smooth");
    return true;
  }

  size_t horizon = result->x.size();

  // lateral and longitudinal constraints
  const double lateral_bound =
      config_.smoother_conf().max_lateral_boundary_bound();

  std::vector<std::pair<double, double>> raw_point2d, smoothed_point2d;
  std::vector<double> bounds;

  raw_point2d.emplace_back(
      std::make_pair(result->x.front(), result->y.front()));
  bounds.push_back(0.0);

  for (size_t i = 1; i < horizon - 1; i++) {
    raw_point2d.emplace_back(std::make_pair(result->x[i], result->y[i]));
    bounds.push_back(lateral_bound);
  }

  raw_point2d.emplace_back(std::make_pair(result->x.back(), result->y.back()));
  bounds.push_back(0.0);

  NormalizePoints(&raw_point2d);

  if (!CosThetaSmooth(raw_point2d, bounds, &smoothed_point2d)) {
    AERROR("Partitioned path smoothing failed");
    return false;
  }

  DeNormalizePoints(&smoothed_point2d);

  // for (size_t i = 0; i < horizon; i++) {
  //   ADEBUG("raw point: " << result->x[i] << ", " << result->y[i]
  //                        << " smoothed_point: " << smoothed_point2d[i].first
  //                        << ", " << smoothed_point2d[i].second);
  // }

  if (smoothed_point2d.size() != horizon) {
    AERROR("Size of smoothed path is not equal to raw path");
    return false;
  }

  if (!planning::DiscretePointsMath::ComputePathPofile(
          smoothed_point2d, &result->phi, &result->accumulated_s,
          &result->kappa, &result->dkappa)) {
    AERROR("Computing path profile failed");
    return false;
  }

  for (size_t i = 0; i < horizon; i++) {
    result->x[i] = smoothed_point2d[i].first;
    result->y[i] = smoothed_point2d[i].second;
    result->phi[i] = common::math::NormalizeAngle(
        result->phi[i] + (result->gear[i] ? 0.0 : M_PI));
  }

  ADEBUG("Generating smoothed partitioned path done");
  return true;
}

void StaticPathGenerator::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;

  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

void StaticPathGenerator::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

bool StaticPathGenerator::CosThetaSmooth(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  // box contraints on pos are used in cos theta smoother, thus shrink the
  // bounds by 1.0 / sqrt(2.0)
  std::vector<double> box_bounds = bounds;
  const double box_ratio = M_SQRT1_2;
  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  std::vector<double> opt_x;
  std::vector<double> opt_y;

  if (!cos_theta_smoother_->Solve(raw_point2d, box_bounds, &opt_x, &opt_y)) {
    AERROR("cos theta smoothing failed");
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    AERROR("return by costheta smoother is wrong. size smaller than 2");
    return false;
  }

  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

  std::size_t point_size = opt_x.size();
  for (std::size_t i = 0; i < point_size; i++) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

std::pair<int, int> StaticPathGenerator::QueryNearestPointByPoistion(
    const double x, const double y, const size_t index) const {
  ADEBUG("current index: " << index);
  ADEBUG("gear size: " << path_.gear.size());

  const bool init_gear = path_.gear[index];

  double min_dist = std::numeric_limits<double>::infinity();
  double min_index = index;
  bool curr_gear = path_.gear[index];

  int i = static_cast<int>(index);

  while (init_gear == curr_gear && i > 0) {
    curr_gear = path_.gear[i--];
  }

  while (init_gear == curr_gear && i < static_cast<int>(path_length_ - 1)) {
    double dist = std::hypot(path_.x[i] - x, path_.y[i] - y);
    if (dist < min_dist) {
      min_dist = dist;
      min_index = i;
    }
    curr_gear = path_.gear[++i];
  }

  return std::make_pair(min_index, i);
}

StaticPathResult StaticPathGenerator::GenerateLocalProfile(const double x,
                                                           const double y) {
  const auto matched_point =
      QueryNearestPointByPoistion(x, y, current_start_index_);

  const int start_index = matched_point.first;
  const int end_index = matched_point.second;
  const int length = end_index - start_index;

  if ((length == 1) && (end_index != path_length_)) {
    current_start_index_ = end_index;
  } else {
    current_start_index_ = start_index;
  }

  ADEBUG("path_length_: " << path_length_);

  ADEBUG("start_index: " << start_index);
  ADEBUG("end_index: " << end_index);

  StaticPathResult stitched_result;

  const int first = std::max<int>(start_index - 1, 0);
  const int last = std::min<int>(start_index + 20, end_index - 1);

  ADEBUG("first: " << first);
  ADEBUG("last: " << last);

  std::copy(path_.x.begin() + first, path_.x.begin() + last,
            std::back_inserter(stitched_result.x));
  std::copy(path_.y.begin() + first, path_.y.begin() + last,
            std::back_inserter(stitched_result.y));
  std::copy(path_.phi.begin() + first, path_.phi.begin() + last,
            std::back_inserter(stitched_result.phi));
  std::copy(path_.v.begin() + first, path_.v.begin() + last,
            std::back_inserter(stitched_result.v));
  std::copy(path_.a.begin() + first, path_.a.begin() + last,
            std::back_inserter(stitched_result.a));
  std::copy(path_.kappa.begin() + first, path_.kappa.begin() + last,
            std::back_inserter(stitched_result.kappa));
  std::copy(path_.accumulated_s.begin() + first,
            path_.accumulated_s.begin() + last,
            std::back_inserter(stitched_result.accumulated_s));
  std::copy(path_.relative_time.begin() + first,
            path_.relative_time.begin() + last,
            std::back_inserter(stitched_result.relative_time));
  std::copy(path_.gear.begin() + first, path_.gear.begin() + last,
            std::back_inserter(stitched_result.gear));
  return stitched_result;
}

bool StaticPathGenerator::GetTemporalProfile(StaticPathResult* result) {
  std::vector<StaticPathResult> partitioned_results;
  if (!TrajectoryPartition(*result, &partitioned_results)) {
    AERROR("TrajectoryPartition failed");
    return false;
  }
  StaticPathResult stitched_result;
  for (const auto& result : partitioned_results) {
    std::copy(result.x.begin(), result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(result.y.begin(), result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(result.phi.begin(), result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    std::copy(result.v.begin(), result.v.end() - 1,
              std::back_inserter(stitched_result.v));
    std::copy(result.a.begin(), result.a.end() - 1,
              std::back_inserter(stitched_result.a));
    std::copy(result.gear.begin(), result.gear.end() - 1,
              std::back_inserter(stitched_result.gear));
    std::copy(result.relative_time.begin(), result.relative_time.end() - 1,
              std::back_inserter(stitched_result.relative_time));
    std::copy(result.accumulated_s.begin(), result.accumulated_s.end() - 1,
              std::back_inserter(stitched_result.accumulated_s));
    std::copy(result.kappa.begin(), result.kappa.end() - 1,
              std::back_inserter(stitched_result.kappa));
  }
  stitched_result.x.push_back(partitioned_results.back().x.back());
  stitched_result.y.push_back(partitioned_results.back().y.back());
  stitched_result.phi.push_back(partitioned_results.back().phi.back());
  stitched_result.v.push_back(partitioned_results.back().v.back());
  stitched_result.a.push_back(partitioned_results.back().a.back());
  stitched_result.relative_time.push_back(
      partitioned_results.back().relative_time.back());
  stitched_result.gear.push_back(partitioned_results.back().gear.back());
  stitched_result.accumulated_s.push_back(
      partitioned_results.back().accumulated_s.back());
  stitched_result.kappa.push_back(partitioned_results.back().kappa.back());

  *result = stitched_result;
  ADEBUG("GetTemporalProfile done");
  return true;
}

bool StaticPathGenerator::TrajectoryPartition(
    const StaticPathResult& result,
    std::vector<StaticPathResult>* partitioned_results) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() != y.size() || x.size() != y.size()) {
    AERROR("states size are not equal when do trajectory paritioning");
    return false;
  }

  size_t horizon = x.size();
  partitioned_results->clear();
  partitioned_results->emplace_back();
  auto* current_traj = &(partitioned_results->back());
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
      current_traj->gear.push_back(current_gear);
      partitioned_results->emplace_back();
      current_traj = &(partitioned_results->back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
    current_traj->gear.push_back(current_gear);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());
  current_traj->gear.push_back(current_gear);

  for (auto& result : *partitioned_results) {
    if (!GenerateSmoothPathProfle(&result)) {
      AWARN("Smoothing failed. Using raw partitioned path");
    }

    if (!FLAGS_use_s_curve_speed_smooth) {
      if (!GenerateSpeedAcceleration(&result)) {
        AERROR("GenerateSpeedAcceleration failed");
        return false;
      }
    } else {
      if (!GenerateSCurveSpeedAcceleration(&result)) {
        AERROR("GenerateSCurveSpeedAcceleration failed");
        return false;
      }
    }
  }
  ADEBUG("TrajectoryPartition done");
  return true;
}

bool StaticPathGenerator::GenerateSpeedAcceleration(StaticPathResult* result) {
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR("result size check when generating speed and acceleration failed");
    return false;
  }
  const size_t horizon = result->x.size();
  const double dt = config_.delta_t();

  result->v.clear();
  result->a.clear();
  result->relative_time.clear();

  // load velocity from position
  // initial and end speed are set to be zeros
  result->v.push_back(0.0);
  for (size_t i = 1; i < horizon - 1; i++) {
    double discrete_v =
        (((result->x[i + 1] - result->x[i]) / dt) * std::cos(result->phi[i]) +
         ((result->x[i] - result->x[i - 1]) / dt) * std::cos(result->phi[i])) /
            2.0 +
        (((result->y[i + 1] - result->y[i]) / dt) * std::sin(result->phi[i]) +
         ((result->y[i] - result->y[i - 1]) / dt) * std::sin(result->phi[i])) /
            2.0;
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  for (size_t i = 0; i < horizon - 1; i++) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / dt;
    result->a.push_back(discrete_a);
  }
  result->a.push_back(0.0);

  result->relative_time.push_back(0.0);
  for (size_t i = 1; i < horizon; i++) {
    double time = dt + result->relative_time[i - 1];
    result->relative_time.push_back(time);
  }

  return true;
}

bool StaticPathGenerator::GenerateSCurveSpeedAcceleration(
    StaticPathResult* result) {
  CHECK_NOTNULL(result);
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR("result size check when generating relative time failed");
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    AERROR("result sizes are not equal");
  }

  double init_heading = result->phi.front();
  const Vec2d init_tracking_vec(result->x[1] - result->x[0],
                                result->y[1] - result->y[0]);

  const bool gear = std::abs(common::math::NormalizeAngle(
                        init_heading - init_tracking_vec.Angle())) < M_PI_2;

  size_t path_points_size = result->x.size();

  double accumulated_s = 0.0;
  result->accumulated_s.resize(path_points_size);

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

  const double init_v = 0.0;
  const double init_a = 0.0;

  ADEBUG(config_.DebugString());

  const double max_forward_v = config_.optimizer_conf().max_forward_velocity();
  const double max_reverse_v = config_.optimizer_conf().max_reverse_velocity();
  const double max_forward_a =
      config_.optimizer_conf().max_forward_acceleration();
  const double max_reverse_a =
      config_.optimizer_conf().max_reverse_acceleration();
  const double max_acc_jerk = config_.optimizer_conf().max_acceleration_jerk();
  const double dt = config_.delta_t();
  const double time_looseness_ratio = config_.time_looseness_ratio();
  const double max_path_time = config_.max_path_time();

  SpeedData speed_data;

  const double path_length = result->accumulated_s.back();
  const double total_time = std::max(
      gear ? time_looseness_ratio *
                 (max_forward_v * max_forward_v + path_length * max_forward_a) /
                 (max_forward_v * max_forward_a)
           : time_looseness_ratio *
                 (max_reverse_v * max_reverse_v + path_length * max_reverse_a) /
                 (max_reverse_a * max_reverse_v),
      max_path_time);

  const size_t num_of_knots = static_cast<size_t>(total_time / dt) + 1;

  autoagric::planning::PiecewiseJerkSpeedProblem piecewise_jerk_problem(
      num_of_knots, dt, {0.0, std::abs(init_v), std::abs(init_a)});

  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, path_length});
  const double max_v = gear ? max_forward_v : max_reverse_v;
  const double max_a = gear ? max_forward_a : max_reverse_a;

  const auto upper_dx = std::max(max_v, std::abs(init_v));
  const auto upper_ddx = std::max(max_a, std::abs(init_a));

  std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                   {0.0, upper_dx});
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {-upper_ddx, upper_ddx});

  x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
  // dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
  // ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

  std::vector<double> x_ref(num_of_knots, path_length);

  const double weight_x_ref = config_.optimizer_conf().weight_x_ref();
  const double weight_ddx = config_.optimizer_conf().weight_ddx();
  const double weight_dddx = config_.optimizer_conf().weight_dddx();

  // Eigen::MatrixXd P = Eigen::MatrixXd::Zero(num_of_knots * 3, num_of_knots * 3);

  // size_t n = num_of_knots;

  // for (size_t i = 0; i < n; ++i) {
  //   P(i, i) = weight_x_ref;
  // }

  // // x(i)'^2 * (w_dx_ref + penalty_dx)
  // for (size_t i = 0; i < n; ++i) {
  //   P(n + i, n + i) = 0.0;
  // }
  // // x(n-1)'^2 * (w_dx_ref + penalty_dx + w_end_dx)

  // auto delta_s_square = dt * dt;
  // // x(i)''^2 * (w_ddx +  w_dddx / delta_s^2)
  // P(2 * n, 2 * n) = weight_ddx + weight_dddx / delta_s_square;

  // // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  // for (size_t i = 1; i < n - 1; ++i) {
  //   P(2 * n + i, 2 * n + i) = weight_ddx + 2.0 * weight_dddx / delta_s_square;
  // }
  // // x(i)''^2 * (w_ddx +  w_dddx / delta_s^2)
  // P(3 * n - 1, 3 * n - 1) = weight_ddx + weight_dddx / delta_s_square;

  // // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  // for (size_t i = 1; i < n; ++i) {
  //   P(2 * n + i, 2 * n + i - 1) = -1.0 * weight_dddx / delta_s_square;
  // }

  // std::vector<double> p_indptr;
  // std::vector<double> p_indices;
  // std::vector<double> p_data;

  // common::math::DenseToCSCMatrix(P, &p_data, &p_indices, &p_indptr);

  // ADEBUG("p_indptr: ");
  // for (size_t i = 0; i < p_indptr.size(); i++) {
  //   ADEBUG(p_indptr[i]);
  // }

  // ADEBUG("p_indices: ");
  // for (size_t i = 0; i < p_indices.size(); i++) {
  //   ADEBUG(p_indices[i]);
  // }

  // ADEBUG("p_data:");
  // for (size_t i = 0; i < p_data.size(); i++) {
  //   ADEBUG(p_data[i] * 2.0);
  // }

  // for (size_t i = 0; i < num_of_knots; i++) {
  //   ADEBUG("x_ref: " << x_ref[i]);
  //   ADEBUG("x_bounds: " << x_bounds[i].first << " " << x_bounds[i].second);
  //   ADEBUG("dx_bounds: " << dx_bounds[i].first << " " <<
  //   dx_bounds[i].second); ADEBUG("ddx_bounds: " << ddx_bounds[i].first << " "
  //                         << ddx_bounds[i].second);
  //
  ADEBUG("num_of_knots " << num_of_knots);
  ADEBUG("path_length " << path_length);
  ADEBUG("upper_dx " << upper_dx);
  ADEBUG("upper_ddx " << upper_ddx);
  ADEBUG("total_time " << total_time);
  ADEBUG("max_acc_jerk " << max_acc_jerk);

  piecewise_jerk_problem.set_x_ref(weight_x_ref, std::move(x_ref));
  piecewise_jerk_problem.set_weight_ddx(weight_ddx);
  piecewise_jerk_problem.set_weight_dddx(weight_dddx);
  piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);
  // piecewise_jerk_problem.set_scale_factor({1.0, 2.0, 3.0});
  // std::array<double, 3> scale_factor_ = {{10.0, 100.0, 10000.0}};

  if (!piecewise_jerk_problem.Optimize(
          config_.optimizer_conf().max_num_of_iterations())) {
    AERROR("Piecewise jerk speed optimizer failed");
    return false;
  }

  const auto& s = piecewise_jerk_problem.opt_x();
  const auto& ds = piecewise_jerk_problem.opt_dx();
  const auto& dds = piecewise_jerk_problem.opt_ddx();

  speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  constexpr double kEpsilon = 1e-6;
  constexpr double sEpsilon = 1e-6;

  for (size_t i = 1; i < num_of_knots; i++) {
    if (s[i - 1] - s[i] > kEpsilon) {
      ADEBUG("unexpected decreasing s in speed acceleration at time "
             << static_cast<double>(i) * dt << " with total time "
             << total_time);
      break;
    }
    speed_data.AppendSpeedPoint(s[i], dt * static_cast<double>(i), ds[i],
                                dds[i], (dds[i] - dds[i - 1]) / dt);
    if (path_length - s[i] < sEpsilon) {
      break;
    }
  }

  ADEBUG(speed_data.DebugString());

  DiscretizedPath path_data;

  for (size_t i = 0; i < path_points_size; i++) {
    common::PathPoint path_point;
    path_point.set_x(result->x[i]);
    path_point.set_y(result->y[i]);
    path_point.set_theta(result->phi[i]);
    path_point.set_kappa(result->kappa[i]);
    path_point.set_s(result->accumulated_s[i]);
    path_data.push_back(std::move(path_point));
  }

  StaticPathResult combined_result;

  const double kDenseTimeResolution = config_.dense_time_resolution();
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
    combined_result.kappa.push_back(path_point.kappa());
    combined_result.relative_time.push_back(cur_rel_time);
    combined_result.v.push_back(gear ? speed_point.v() : -speed_point.v());
    combined_result.a.push_back(gear ? speed_point.a() : -speed_point.a());
    combined_result.gear.push_back(gear);

    TrajectoryPoint debug_point;
    debug_point.mutable_path_point()->Swap(&path_point);
    debug_point.set_a(combined_result.a.back());
    debug_point.set_v(combined_result.v.back());
    debug_point.set_relative_time(combined_result.relative_time.back());

    ADEBUG(debug_point.DebugString());
  }

  *result = combined_result;
  return true;
}

}  // namespace planning
}  // namespace autoagric
