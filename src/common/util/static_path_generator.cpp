#include "common/util/static_path_generator.h"

#include <chrono>

#include "autoagric/common/pnc_point.pb.h"
#include "common/macro.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "planning/common/speed_profile_generator.h"
#include "planning/math/discrete_points_math.h"

namespace autoagric {
namespace common {
namespace util {

namespace {
constexpr double kDoubleEpsilon = 1e-3;
}

using autoagric::planning::CosThetaSmoother;
using autoagric::planning::TrajectorySmootherConfig;
using common::math::Vec2d;

StaticPathGenerator::StaticPathGenerator(
    const std::string& file_path, const bool enable_periodic_speed_profile,
    const double delta_t)
    : file_path_(file_path),
      enable_periodic_speed_profile_(enable_periodic_speed_profile),
      delta_t_(delta_t) {
  AINFO("Load static path from " << file_path);
  AINFO("Using periodic speed profile " << enable_periodic_speed_profile);
  if (enable_periodic_speed_profile) {
    AERROR_IF(delta_t_ < 1e-2, "timestep is too small");
    CHECK_GT(delta_t_, 1e-2);
  }
}

bool StaticPathGenerator::Init(const TrajectorySmootherConfig& config) {
  smoother_config_ = config;

  csv_reader_.reset(new CSVReader());

  csv_reader_->LoadDataFromFile(file_path_.c_str(), &path_);

  if (path_.x.size() != path_.y.size() || path_.x.size() != path_.phi.size()) {
    AERROR("sizes of necessary states of overall path are not equal");
    return false;
  }

  path_length_ = path_.x.size();

  current_start_index_ = 0;

  const auto& cos_theta_config =
      smoother_config_.discrete_points().cos_theta_smoothing();

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
  const double lateral_bound = smoother_config_.max_lateral_boundary_bound();

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
  const double box_ratio = 1.0 / std::sqrt(2.0);
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

  int i = index;

  while (init_gear == curr_gear && i >= 0) {
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

  ADEBUG("i    x     y     h     v     a     k     s     t  ");
  for (size_t i = 0; i < stitched_result.gear.size(); i++) {
    ADEBUG(std::setprecision(2)
           << std::fixed << i << " " << stitched_result.x[i] << " "
           << stitched_result.y[i] << " " << stitched_result.phi[i] << " "
           << stitched_result.v[i] << " " << stitched_result.a[i] << " "
           << stitched_result.kappa[i] << " "
           << stitched_result.accumulated_s[i] << " "
           << stitched_result.relative_time[i]);
  }
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

  ADEBUG("i    x     y     h     v     a     k     s     t  ");
  for (size_t i = 0; i < stitched_result.x.size(); i++) {
    ADEBUG(std::setprecision(2)
           << std::fixed << i << " " << stitched_result.x[i] << " "
           << stitched_result.y[i] << " " << stitched_result.phi[i] << " "
           << stitched_result.v[i] << " " << stitched_result.a[i] << " "
           << stitched_result.kappa[i] << " "
           << stitched_result.accumulated_s[i] << " "
           << stitched_result.relative_time[i]);
  }

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

    if (enable_periodic_speed_profile_) {
      if (!GenerateSpeedAcceleration(&result)) {
        AERROR("GenerateSpeedAcceleration failed");
        return false;
      }
    } else {
      if (!GenerateRelativetimeAcceleration(&result)) {
        AERROR("GenerateRelativeTime failed");
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

  result->v.clear();
  result->a.clear();
  result->relative_time.clear();

  // load velocity from position
  // initial and end speed are set to be zeros
  result->v.push_back(0.0);
  for (size_t i = 1; i < horizon - 1; i++) {
    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                             std::cos(result->phi[i]) +
                         ((result->x[i] - result->x[i - 1]) / delta_t_) *
                             std::cos(result->phi[i])) /
                            2.0 +
                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
                             std::sin(result->phi[i]) +
                         ((result->y[i] - result->y[i - 1]) / delta_t_) *
                             std::sin(result->phi[i])) /
                            2.0;
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  for (size_t i = 0; i < horizon - 1; i++) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }
  result->a.push_back(0.0);

  result->relative_time.push_back(0.0);
  for (size_t i = 1; i < horizon; i++) {
    double time = delta_t_ + result->relative_time[i - 1];
    result->relative_time.push_back(time);
  }

  return true;
}

bool StaticPathGenerator::GenerateRelativetimeAcceleration(
    StaticPathResult* result) {
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR("result size check when generating relative time failed");
    return false;
  }
  result->relative_time.clear();
  result->a.clear();

  const size_t horizon = result->x.size();

  result->relative_time.push_back(0.0);
  for (size_t i = 1; i < horizon; i++) {
    double distance = std::hypot(result->x[i + 1] - result->x[i],
                                 result->y[i + 1] - result->y[i]);
    double discrete_time = distance / (result->v[i + 1] + result->v[i]) * 2.0;
    result->relative_time.push_back(discrete_time);
  }

  // load acceleration from velocity
  for (size_t i = 0; i < horizon - 1; i++) {
    const double discrete_a =
        (result->v[i + 1] - result->v[i]) /
        std::max(kDoubleEpsilon,
                 (result->relative_time[i + 1] - result->relative_time[i]));
    result->a.push_back(discrete_a);
  }
  result->a.push_back(0.0);
  return true;
}

}  // namespace util
}  // namespace common
}  // namespace autoagric