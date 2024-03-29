#pragma once

#include <deque>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/planning/planning.pb.h"
#include "common/math/vec2d.h"

/**
 * @namespace autoargic::control
 * @brief autoagric::control
 */

namespace autoagric {
namespace control {

/**
 * @class TrajectoryAnalyzer
 * @brief process point query and conversion related to trjactory
 * @note trajectory points published by autoagric planning was on "map" frame,
 * even spacing, no time information, while Apollo planning generates trajectory
 * points on vehicle frame, sampled on even time interval. A coordinate
 * conversion is added.
 */

class TrajectoryAnalyzer {
 public:
  /**
   * @brief constructor
   */
  TrajectoryAnalyzer() = default;

  /**
   * @brief constructor
   * @param published_trajectory trajectory data generate by planning module
   * @note modified to update relative time based on point-wise speed and
   * station info
   */
  TrajectoryAnalyzer(
      const planning::ADCTrajectory* planning_published_trajectory);

  /**
   * @brief destructor
   */
  ~TrajectoryAnalyzer() = default;

  /**
   * @brief shallow copy
   */
  TrajectoryAnalyzer& operator=(TrajectoryAnalyzer&& traj) {
    this->header_time_ = traj.header_time_;
    this->seq_num_ = traj.seq_num_;
    this->trajectory_points_ = std::move(traj.trajectory_points_);
    return *this;
  }

  /**
   * @brief get sequence number of trajectory
   * @note deprecated for local planning module (no sequential info)
   */
  size_t seq_num() { return seq_num_; };

  /**
   * @brief query a point of trajectory that its absolute time is closest to the
   * given time
   * @param t absolute time for query
   * @return a point of trajectory
   */
  common::TrajectoryPoint QueryNearestPointByAbsoluteTime(const double t) const;

  /**
   * @brief query a point of trajectory that its relative time is closest to the
   * give time. The time is relative to the first point of trajectory
   * @param t relative time for query
   * @return a point of trajetory
   */
  common::TrajectoryPoint QueryNearestPointByRelativeTime(const double t) const;

  /**
   * @brief query a point of trajectory that its position is closest to the
   * given position
   * @param x value of x-coordination in the given position
   * @param y value of y-coordination in the given position
   * @return a point of trajectory
   */
  common::TrajectoryPoint QueryNearestPointByPoistion(const double x,
                                                      const double y) const;

  /**
   * @brief qurey a point on trajectory that its positin is closest to the given
   * position
   * @param x value of x-coordination in the given position
   * @param y value of y-coofdination in the given position
   * @return a point on trajectory. The point may be a point of trajectory or
   * interpolated by two adjacent points of trajectory
   */
  common::PathPoint QueryMatchedPathPoint(const double x, const double y) const;
  /**
   * @brief qurey a point on trajectory that its positin is closest to the given
   * position
   * @param x value of x-coordination in the given position
   * @param y value of y-coofdination in the given position
   * @return a point on trajectory. The point may be a point of trajectory or
   * interpolated by two adjacent points of trajectory
   */
  common::TrajectoryPoint QueryMatchedTrajectoryPoint(const double x,
                                                      const double y) const;

  /**
   * @brief convert a position with theta and speed to trajectory frame,
   * longitudinal and lateral direction to the trajectory
   * @param x x-value of the position
   * @param y y-value of the position
   * @param theta heading angle on the position
   * @param v speed on the position
   * @param matched_point matched point on trajectory for the given position
   * @param ptr_s longitudinal distance
   * @param ptr_s_dot longitudinal speed
   * @param ptr_d lateral distance
   * @param ptr_d_dot lateral speed
   */
  void ToTrajectoryFrame(const double x, const double y, const double theta,
                         const double v, const common::PathPoint& matched_point,
                         double* ptr_s, double* ptr_s_dot, double* ptr_d,
                         double* ptr_d_dot) const;

  /**
   * @brief Transform the current trajectory points to the center of mass (COM)
   * of the vehicle, given the distance from rear wheels to the center of mass.
   * @param rear_to_com_distance distance from rear wheels to the vehicle's
   * center of mass
   */
  void TrajectoryTransformToCOM(const double rear_to_com_distance);

  /**
   * @brief compute the position of center of mass( COM) of the vehicle, given
   * the distance from rear wheels to the center of mass
   * @param rear_to_com_distance distance from rear wheels to the vehicle's
   * center of mass
   * @param path_point PathPoint along the published planning trajectory
   * @return the position of the vehicle's center of mass
   */
  common::math::Vec2d ComputeCOMPosition(
      const double rear_to_com_distance,
      const common::PathPoint& path_point) const;

  /**
   * @brief convert a position to trajectory frame
   * @param ref_point origin of the trajectory frame to cenvert to
   * @param x x-value of the position
   * @param y y-value of the position
   * @note depracated for lat_controller
   */
  void ToTrajectoryFrame(const common::PathPoint& ref_point, const double x,
                         const double y) const;

  /**
   * @brief get all points of trjactory
   * @return a vector of trajectory points
   */
  const std::vector<common::TrajectoryPoint>& trajectory_points() const;

  /**
   * @brief re-sample trajectory points on relative time
   * @param start_time sampling starting time
   * @param dt sampling time step
   * @param resampled_trajectory result trajectory that sampled on isometric
   * time step
   * @note deprecate for lat_controller
   */
  void SampleByRelativeTime(
      const double start_time, const double dt, const size_t trajectory_size,
      std::vector<common::TrajectoryPoint>& resampled_trajectory) const;

  /**
   * @brief interpolat trajectory points by relative time
   * @param start_time  starting time
   * @param dt time step
   * @param trajectory_size number of points on interpolated trajectory
   * @return result trajectory that sampled on isometric time step
   * @note deprecate for lat_controller
   */
  std::vector<common::TrajectoryPoint> InterpolateByTime(
      const double start_time, const double dt,
      const size_t trajectory_size) const;
  /**
   * @brief convert Cartesian coordinates of trajectory points to Frenet
   * coordinates
   * @param rx x Cartesian coordinate of Frenet origin
   * @param ry y Cartesian coordinate of Frenet origin
   * @param rtheta heading angle Cartesian coordinate of Frenet origin
   * @param ptr_trajectory_points trajectory in Cartesian coordinates
   * @note deprecate for lat_controller
   */
  static std::vector<common::TrajectoryPoint> ToLoc(
      const double rx, const double ry, const double rtheta,
      const std::vector<common::TrajectoryPoint>* ptr_trajectory_points);

  /**
   * @brief convert Frenet coordinates of trajectory points to Cartesian
   * coordinates
   * @param rx x Cartesian coordinate of Frenet origin
   * @param ry y Cartesian coordinate of Frenet origin
   * @param rtheta heading angle Cartesian coordinate of Frenet origin
   * @param ptr_trajectory_points trajectory in Frenet coordinates
   * @return vector of trajectory points in Cartesian coordinates
   * @note deprecate for lat_controller
   */
  static std::vector<common::TrajectoryPoint> ToMap(
      const double rx, const double ry, const double rtheta,
      const std::vector<common::TrajectoryPoint>* ptr_trajectory_points);

  template <typename T>
  static common::math::Vec2d Map2Loc(const double rx, const double ry,
                                     const double rtheta, const T& map_point) {
    auto dx = map_point.x() - rx;
    auto dy = map_point.y() - ry;
    auto cos_theta_r = std::cos(rtheta);
    auto sin_theta_r = std::sin(rtheta);
    auto x = cos_theta_r * dy - sin_theta_r * dx;
    auto y = sin_theta_r * dy + cos_theta_r * dx;
    return common::math::Vec2d(x, y);
  }

  template <typename T>
  static common::math::Vec2d Loc2Map(const double rx, const double ry,
                                     const double rtheta, const T& loc_point) {
    auto cos_theta_r = std::cos(rtheta);
    auto sin_theta_r = std::sin(rtheta);

    auto x = loc_point.x();
    auto y = loc_point.y();

    auto x_map = cos_theta_r * y - sin_theta_r * x + rx;
    auto y_map = sin_theta_r * y + cos_theta_r * x + ry;
    return common::math::Vec2d(x_map, y_map);
  }

 private:
  common::TrajectoryPoint FindMinDistancePoint(
      const common::TrajectoryPoint& p0, const common::TrajectoryPoint& p1,
      const double x, const double y) const;

  std::vector<common::TrajectoryPoint> trajectory_points_;

  double header_time_;
  size_t seq_num_ = 0;
};
}  // namespace control
}  // namespace autoagric