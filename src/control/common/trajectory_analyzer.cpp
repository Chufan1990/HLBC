#include "control/common/trajectory_analyzer.h"

#include <memory>

#include "common/macro.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/search.h"
#include "control/common/control_gflags.h"
#include "control/common/interpolation_1d.h"

namespace autoagric {
namespace control {

using common::PathPoint;
using common::TrajectoryPoint;
using common::math::Vec2d;

namespace {

double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
  const double dx = point.path_point().x() - x;
  const double dy = point.path_point().y() - y;
  return dx * dx + dy * dy;
}

PathPoint TrajectoryPointToPathPoint(const TrajectoryPoint &point) {
  if (point.has_path_point()) {
    return point.path_point();
  } else {
    return PathPoint();
  }
}

}  // namespace

TrajectoryAnalyzer::TrajectoryAnalyzer(
    const planning::ADCTrajectory *planning_published_trajectory) {
  header_time_ = planning_published_trajectory->header().timestamp_sec();
  seq_num_ = planning_published_trajectory->header().sequence_num();

  for (int i = 0; i < planning_published_trajectory->trajectory_point_size();
       ++i) {
    trajectory_points_.push_back(
        planning_published_trajectory->trajectory_point(i));
  }
}

PathPoint TrajectoryAnalyzer::QueryMatchedPathPoint(const double x,
                                                    const double y) const {
  return TrajectoryPointToPathPoint(QueryMatchedTrajectoryPoint(x, y));
}

TrajectoryPoint TrajectoryAnalyzer::QueryMatchedTrajectoryPoint(
    const double x, const double y) const {
  CHECK_GT(trajectory_points_.size(), 0U);

  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 0; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  size_t index_start = index_min == 0 ? index_min : index_min - 1;
  size_t index_end =
      index_min + 1 == trajectory_points_.size() ? index_min : index_min + 1;

  const double kEpsilon = 0.001;
  if (index_start == index_end ||
      std::fabs(trajectory_points_[index_start].path_point().s() -
                trajectory_points_[index_end].path_point().s()) <= kEpsilon) {
    return trajectory_points_[index_start];
  }

  return FindMinDistancePoint(trajectory_points_[index_start],
                              trajectory_points_[index_end], x, y);
}

// reference: Optimal trajectory generation for dynamic street scenarios in a
// Frenét Frame,
// Moritz Werling, Julius Ziegler, Sören Kammel and Sebastian Thrun, ICRA 2010
// similar to the method in this paper without the assumption the "normal"
// vector
// (from vehicle position to ref_point position) and reference heading are
// perpendicular.
void TrajectoryAnalyzer::ToTrajectoryFrame(const double x, const double y,
                                           const double theta, const double v,
                                           const PathPoint &ref_point,
                                           double *ptr_s, double *ptr_s_dot,
                                           double *ptr_d,
                                           double *ptr_d_dot) const {
  double dx = x - ref_point.x();
  double dy = y - ref_point.y();

  double cos_ref_theta = std::cos(ref_point.theta());
  double sin_ref_theta = std::sin(ref_point.theta());

  // the sin of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
  *ptr_d = cross_rd_nd;

  // the cos of diff angle between vector (cos_ref_theta, sin_ref_theta) and
  // (dx, dy)
  double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
  *ptr_s = ref_point.s() + dot_rd_nd;

  double delta_theta = theta - ref_point.theta();
  double cos_delta_theta = std::cos(delta_theta);
  double sin_delta_theta = std::sin(delta_theta);

  *ptr_d_dot = v * sin_delta_theta;

  double one_minus_kappa_r_d = 1 - ref_point.kappa() * (*ptr_d);
  if (one_minus_kappa_r_d <= 0.0) {
    AERROR(
        "TrajectoryAnalyzer::ToTrajectoryFrame "
        "found fatal reference and actual difference. "
        "Control output might be unstable:"
        << " ref_point.kappa:" << ref_point.kappa()
        << " ref_point.x:" << ref_point.x() << " ref_point.y:" << ref_point.y()
        << " car x:" << x << " car y:" << y << " *ptr_d:" << *ptr_d
        << " one_minus_kappa_r_d:" << one_minus_kappa_r_d);
    // currently set to a small value to avoid control crash.
    one_minus_kappa_r_d = 0.01;
  }

  *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByAbsoluteTime(
    const double t) const {
  return QueryNearestPointByRelativeTime(t - header_time_);
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByRelativeTime(
    const double t) const {
  auto func_comp = [](const TrajectoryPoint &point,
                      const double relative_time) {
    return point.relative_time() < relative_time;
  };

  auto it_low = std::lower_bound(trajectory_points_.begin(),
                                 trajectory_points_.end(), t, func_comp);

  if (it_low == trajectory_points_.begin()) {
    return trajectory_points_.front();
  }

  if (it_low == trajectory_points_.end()) {
    return trajectory_points_.back();
  }

  if (FLAGS_query_forward_time_point_only) {
    return *it_low;
  } else {
    auto it_lower = it_low - 1;
    if (it_low->relative_time() - t < t - it_lower->relative_time()) {
      return *it_low;
    }
    return *it_lower;
  }
}

TrajectoryPoint TrajectoryAnalyzer::QueryNearestPointByPoistion(
    const double x, const double y) const {
  double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
  size_t index_min = 0;

  for (size_t i = 0; i < trajectory_points_.size(); ++i) {
    double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return trajectory_points_[index_min];
}

const std::vector<TrajectoryPoint> &TrajectoryAnalyzer::trajectory_points()
    const {
  return trajectory_points_;
}

TrajectoryPoint TrajectoryAnalyzer::FindMinDistancePoint(
    const TrajectoryPoint &p0, const TrajectoryPoint &p1, const double x,
    const double y) const {
  // given the fact that the discretized trajectory is dense enough,
  // we assume linear trajectory between consecutive trajectory points.
  auto dist_square = [&p0, &p1, &x, &y](const double s) {
    double px = common::math::lerp(p0.path_point().x(), p0.path_point().s(),
                                   p1.path_point().x(), p1.path_point().s(), s);
    double py = common::math::lerp(p0.path_point().y(), p0.path_point().s(),
                                   p1.path_point().y(), p1.path_point().s(), s);
    double dx = px - x;
    double dy = py - y;
    return dx * dx + dy * dy;
  };

  TrajectoryPoint p = p0;
  double s = common::math::GoldenSectionSearch(dist_square, p0.path_point().s(),
                                               p1.path_point().s());
  p.mutable_path_point()->set_s(s);
  p.mutable_path_point()->set_x(
      common::math::lerp(p0.path_point().x(), p0.path_point().s(),
                         p1.path_point().x(), p1.path_point().s(), s));
  p.mutable_path_point()->set_y(
      common::math::lerp(p0.path_point().y(), p0.path_point().s(),
                         p1.path_point().y(), p1.path_point().s(), s));
  p.mutable_path_point()->set_theta(
      common::math::slerp(p0.path_point().theta(), p0.path_point().s(),
                          p1.path_point().theta(), p1.path_point().s(), s));
  // approximate the curvature at the intermediate point
  p.mutable_path_point()->set_kappa(
      common::math::lerp(p0.path_point().kappa(), p0.path_point().s(),
                         p1.path_point().kappa(), p1.path_point().s(), s));
  p.set_v(common::math::lerp(p0.v(), p0.path_point().s(), p1.v(),
                             p1.path_point().s(), s));
  p.set_a(common::math::lerp(p0.a(), p0.path_point().s(), p1.a(),
                             p1.path_point().s(), s));
  p.set_relative_time(
      common::math::lerp(p0.relative_time(), p0.path_point().s(),
                         p1.relative_time(), p1.path_point().s(), s));
  return p;
}

void TrajectoryAnalyzer::TrajectoryTransformToCOM(
    const double rear_to_com_distance) {
  CHECK_GT(trajectory_points_.size(), 0U);
  for (size_t i = 0; i < trajectory_points_.size(); ++i) {
    auto com = ComputeCOMPosition(rear_to_com_distance,
                                  trajectory_points_[i].path_point());
    trajectory_points_[i].mutable_path_point()->set_x(com.x());
    trajectory_points_[i].mutable_path_point()->set_y(com.y());
  }
}

Vec2d TrajectoryAnalyzer::ComputeCOMPosition(
    const double rear_to_com_distance, const PathPoint &path_point) const {
  // Initialize the vector for coordinate transformation of the position
  // reference point
  Eigen::Vector3d v;
  const double cos_heading = std::cos(path_point.theta());
  const double sin_heading = std::sin(path_point.theta());
  v << rear_to_com_distance * cos_heading, rear_to_com_distance * sin_heading,
      0.0;
  // Original position reference point at center of rear-axis
  Eigen::Vector3d pos_vec(path_point.x(), path_point.y(), path_point.z());
  // Transform original position with vector v
  Eigen::Vector3d com_pos_3d = v + pos_vec;
  // Return transfromed x and y
  return Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

std::vector<TrajectoryPoint> TrajectoryAnalyzer::ToLoc(
    const double rx, const double ry, const double rtheta,
    const std::vector<TrajectoryPoint> *ptr_trajectory_points) {
  std::vector<TrajectoryPoint> ret(ptr_trajectory_points->begin(),
                                   ptr_trajectory_points->end());
  std::for_each(begin(ret), end(ret), [&rx, &ry, &rtheta](TrajectoryPoint &p) {
    auto pos_loc = TrajectoryAnalyzer::Map2Loc(rx, ry, rtheta, p.path_point());
    auto theta_loc =
        common::math::NormalizeAngle(p.path_point().theta() - rtheta);
    p.mutable_path_point()->set_x(pos_loc.x());
    p.mutable_path_point()->set_y(pos_loc.y());
    p.mutable_path_point()->set_theta(theta_loc);
  });
  return ret;
}

std::vector<TrajectoryPoint> TrajectoryAnalyzer::ToMap(
    const double rx, const double ry, const double rtheta,
    const std::vector<TrajectoryPoint> *ptr_trajectory_points) {
  std::vector<TrajectoryPoint> ret(ptr_trajectory_points->begin(),
                                   ptr_trajectory_points->end());
  std::for_each(begin(ret), end(ret), [&rx, &ry, &rtheta](TrajectoryPoint &p) {
    auto pos_map = TrajectoryAnalyzer::Loc2Map(rx, ry, rtheta, p.path_point());
    auto theta_map =
        common::math::NormalizeAngle(p.path_point().theta() + rtheta);

    p.mutable_path_point()->set_x(pos_map.x());
    p.mutable_path_point()->set_y(pos_map.y());
    p.mutable_path_point()->set_theta(theta_map);
  });
  return ret;
}

std::vector<TrajectoryPoint> TrajectoryAnalyzer::InterpolateByTime(
    const double start_time, const double dt,
    const size_t trajectory_size) const {
  if (trajectory_points_.size() == 0) {
    AERROR("Empty original trajectory");
    return std::vector<TrajectoryPoint>(0);
  }

  if (trajectory_points_.size() == 1) {
    AWARN("End point on trajectory");
    return std::vector<TrajectoryPoint>(trajectory_size,
                                        trajectory_points_.front());
  }

  std::vector<TrajectoryPoint> resampled_trajectory;
  resampled_trajectory.resize(trajectory_size);

  auto func_comp = [](const TrajectoryPoint &point,
                      const double relative_time) {
    return point.relative_time() < relative_time;
  };

  auto p0 = trajectory_points_.begin();
  auto p1 = trajectory_points_.begin();
  double t = start_time;

  for (auto &p : resampled_trajectory) {
    p1 = std::lower_bound(p1, trajectory_points_.end(), t, func_comp);

    p1 = p1 == trajectory_points_.end() ? p1 - 1 : p1;
    p0 = p1 == trajectory_points_.begin() ? p1 : p1 - 1;

    auto path_point = p.mutable_path_point();
    path_point->set_s(
        common::math::lerp(p0->path_point().s(), p0->relative_time(),
                           p1->path_point().s(), p1->relative_time(), t));
    path_point->set_x(
        common::math::lerp(p0->path_point().x(), p0->relative_time(),
                           p1->path_point().x(), p1->relative_time(), t));
    path_point->set_y(
        common::math::lerp(p0->path_point().y(), p0->relative_time(),
                           p1->path_point().y(), p1->relative_time(), t));
    path_point->set_theta(
        common::math::slerp(p0->path_point().theta(), p0->relative_time(),
                            p1->path_point().theta(), p1->relative_time(), t));
    path_point->set_kappa(
        common::math::lerp(p0->path_point().kappa(), p0->relative_time(),
                           p1->path_point().kappa(), p1->relative_time(), t));
    p.set_a(common::math::lerp(p0->a(), p0->relative_time(), p1->a(),
                               p1->relative_time(), t));
    p.set_v(common::math::lerp(p0->v(), p0->relative_time(), p1->v(),
                               p1->relative_time(), t));
    p.set_relative_time(t - start_time);

    t += dt;
  }

  return resampled_trajectory;
}

}  // namespace control
}  // namespace autoagric