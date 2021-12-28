#pragma once

#include <vector>

#include "autoagric/common/pnc_point.pb.h"
#include "common/math/box2d.h"
#include "planning/common/obstacle.h"

namespace autoagric {
namespace planning {

class CollisionChecker {
 public:
  CollisionChecker(
      const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
      const dobule ego_vehicle_d,
      const std::vector<common::PathPoint>& discretized_reference_line,
      const ReferenceLineInfo* ptr_reference_line_info,
      const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph);

  bool IsCollision(const DiscretizedTrajectory& discretized_trajectory);

  static bool IsCollision(const std::vector<const Obstacle*>& obstacles,
                          const DiscretizedTrajectory& ego_trajectory,
                          const double ego_length, const double ego_width,
                          const double ego_edge_to_center);

 private:
  void BuildPredictedEnvironment(
      const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
      const double ego_vehicle_d,
      const std::vector<common::PathPoint>& discretized_reference_line);

  bool IsEgoVehicleInLane(const double ego_vehicle_s,
                          const double ego_vehicle_d);

  bool IsObstacleBehindEgoVehicle(
      const Obstacle* obstacle, const double ego_vehicle_s,
      const double ego_vehicle_d,
      std::vector<common::PathPoint>& discretized_reference_line);

 private:
  const ReferenceLineInfo* ptr_reference_line_info_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
  std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles_;
};

}  // namespace planning
}  // namespace autoagric