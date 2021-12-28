#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "autoagric/perception/perception_obstacle.pb.h"
#include "autoagric/planning/decision.pb.h"
#include "autoagric/planning/sl_boundary.pb.h"
#include "autoagric/prediction/feature.pb.h"
#include "autoagric/prediction/prediction_obstacle.pb.h"
#include "common/math/box2d.h"
#include "common/math/polygon2d.h"

namespace autoagric {
namespace planning {

class Obstacle {
 public:
  Obstacle() = default;
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const prediction::ObstaclePriority::Priority& obstacle_priority,
           const bool is_static);
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const prediction::Trajectory& trajectory,
           const prediction::ObstaclePriority::Priority& obstacle_priority,
           const bool is_static);

  const std::string& Id() const { return id_; }
  void SetId(const std::string& id) { id_ = id; }

  double speed() const { return speed_; }

  int32_t PerceptionId() const { return perception_id_; }

  bool IsStatic() const { return is_static_; }
  bool IsVirtual() const { return is_virtual_; }

  common::TrajectoryPoint GetPointAtTime(const double time) const;

  common::math::Box2d GetBoundingBox(
      const common::TrajectoryPoint& point) const;

  const common::math::Box2d& PerceptionBoundingBox() const {
    return perception_bounding_box_;
  }
  const common::math::Polygon2d& PerceptionPolygon() const {
    return perception_polygon_;
  }
  const prediction::Trajectory& Trajectory() const { return trajectory_; }
  common::TrajectoryPoint* AddTrajectoryPoint() {
    return trajectory_.add_trajectory_point();
  }
  bool HasTrajectory() const {
    return !(trajectory_.trajectory_point().empty());
  }

  const perception::PerceptionObstacle& Perception() const {
    return perception_obstacle_;
  }

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const prediction::PredictionObstacles& predictions);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::string& id, const common::math::Box2d& obstacle_box);

  static bool IsValidPerceptionObstacle(
      const perception::PerceptionObstacle& obstacle);

  static bool IsValidTrajectoryPoint(const common::TrajectoryPoint& point);

  inline bool IsCautionLevelObstacle() const {
    return is_caution_level_obstacle_;
  }

  // const Obstacle* obstacle() const;

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const ObjectDecisionType& LateralDecision() const;

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const ObjectDecisionType& LongitudinalDecision() const;

  std::string DebugString() const;

  const SLBoundary& PerceptionSLBoundary() const;

  const STBoundary& reference_line_st_boundary() const;

  const STBoundary& path_st_boundary() const;

  const std::vector<std::string>& decider_tags() const;

  const std::vector<ObjectDecisionType>& decisions() const;

  void AddLongitudinalDecision(const std::string& decider_tag,
                               const ObjectDecisionType& decision);

  void AddLateralDecision(const std::string& decider_tag,
                          const ObjectDecisionType& decision);
  bool HasLateralDecision() const;

  void set_path_st_boundary(const STBoundary& boundary);

  bool is_path_st_boundary_initialized() {
    return path_st_boundary_initialized_;
  }

  void SetStBoundaryType(const STBoundary::BoundaryType type);

  void EraseStBoundary();

  void SetReferenceLineStBoundary(const STBoundary& boundary);

  void SetReferenceLineStBoundaryType(const STBoundary::BoundaryType type);

  void EraseReferenceLineStBoundary();

  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;

  /**
   * @brief Calculate stop distance with the obstacle using the ADC's minimum
   * turning radius
   */
  double MinRadiusStopDistance(const common::VehicleParam& vehicle_param) const;

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;

  void BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                    const double adc_start_s);

  void SetPerceptionSlBoundary(const SLBoundary& sl_boundary);

  /**
   * @brief check if an ObjectDecisionType is a longitudinal decision.
   **/
  static bool IsLongitudinalDecision(const ObjectDecisionType& decision);

  /**
   * @brief check if an ObjectDecisionType is a lateral decision.
   **/
  static bool IsLateralDecision(const ObjectDecisionType& decision);

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }

  /*
   * @brief IsLaneBlocking is only meaningful when IsStatic() == true.
   */
  bool IsLaneBlocking() const { return is_lane_blocking_; }
  void CheckLaneBlocking(const ReferenceLine& reference_line);
  bool IsLaneChangeBlocking() const { return is_lane_change_blocking_; }
  void SetLaneChangeBlocking(const bool is_distance_clear);

 private:
  bool IsValidObstacle(
      const perception::PerceptionObstacle& perception_obstacle);

 private:
  std::string id_;
  int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_virtual_ = false;
  double speed_ = 0.0;

  bool path_st_boundary_initialized_ = false;

  prediction::Trajectory trajectory_;
  perception::PerceptionObstacle perception_obstacle_;
  common::math::Box2d perception_bounding_box_;
  common::math::Polygon2d perception_polygon_;

  std::vector<ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;
  SLBoundary sl_boundary_;

  STBoundary reference_line_st_boundary_;
  STBoundary path_st_boundary_;

  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;

  // for keep_clear usage only
  bool is_blocking_obstacle_ = false;

  bool is_lane_blocking_ = false;

  bool is_lane_change_blocking_ = false;

  bool is_caution_level_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;
};

}  // namespace planning
}  // namespace autoagric