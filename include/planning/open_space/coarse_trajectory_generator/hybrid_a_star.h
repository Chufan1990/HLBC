#pragma once

#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "autoagric/planning/planner_open_space_config.pb.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/math/vec2d.h"
#include "planning/open_space/coarse_trajectory_generator/grid_search.h"
#include "planning/open_space/coarse_trajectory_generator/node3d.h"
#include "planning/open_space/coarse_trajectory_generator/reeds_shepp_path.h"

namespace autoagric {
namespace planning {

struct HybridAStarResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> accumulated_s;
};

class HybridAStar {
 public:
  explicit HybridAStar(const PlannerOpenSpaceConfig& open_space_conf);
  virtual ~HybridAStar() = default;

  bool Plan(const double sx, const double sy, const double sphi,
            const double ex, const double ey, const double ephi,
            const std::vector<double>& XYbounds,
            const std::vector<std::vector<common::math::Vec2d>>&
                obstacles_vertices_vec,
            HybridAStarResult* result);
  bool TrajectoryPartition(const HybridAStarResult& result,
                           std::vector<HybridAStarResult>* partitioned_result);

  const std::unordered_map<std::string, std::shared_ptr<Node2d>>& DpMap()
      const {
    return dp_map_;
  }

 private:
  bool AnalyticExpansion(std::shared_ptr<Node3d> current_node);

  bool ValidityCheck(std::shared_ptr<Node3d> node);

  bool RSPCheck(const std::shared_ptr<ReedsSheppPath> reeds_shepp_to_end);

  std::shared_ptr<Node3d> LoadRSPinCS(
      const std::shared_ptr<ReedsSheppPath> reeds_shepp_to_end,
      std::shared_ptr<Node3d> current_node);

  std::shared_ptr<Node3d> Next_node_generator(
      std::shared_ptr<Node3d> current_node, size_t next_node_index);
  void CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                         std::shared_ptr<Node3d> next_node);
  double TrajCost(std::shared_ptr<Node3d> current_node,
                  std::shared_ptr<Node3d> next_node);

  double HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node);

  bool GetResult(HybridAStarResult* result);
  bool GetTemporalProfile(HybridAStarResult* result);
  bool GenerateSpeedAcceleration(HybridAStarResult* result);
  bool GenerateSCurveSpeedAcceleration(HybridAStarResult* result);

 private:
  PlannerOpenSpaceConfig planner_open_space_config_;
  common::VehicleParam vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  size_t next_node_num_ = 0;
  double max_steer_angle_ = 0.0;
  double step_size_ = 0.0;
  double xy_grid_resolution_ = 0.0;
  double delta_t_ = 0.0;
  double traj_forward_penalty_ = 0.0;
  double traj_backward_penalty_ = 0.0;
  double traj_gear_switch_penalty_ = 0.0;
  double traj_steer_penalty_ = 0.0;
  double traj_steer_change_penalty_ = 0.0;
  double heu_rs_forward_penalty_ = 0.0;
  double heu_rs_backward_penalty_ = 0.0;
  double heu_rs_steer_penalty_ = 0.0;
  double heu_rs_steer_change_penalty_ = 0.0;
  double longitudinal_safety_margin_ = 0.0;
  double lateral_safety_margin_ = 0.0;
  std::vector<double> XYbounds_;
  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::shared_ptr<Node3d> final_node_;
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;

  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };

  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq_;

  std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;
  std::unique_ptr<ReedsShepp> reeds_shepp_generator_;
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;

  std::unordered_map<std::string, std::shared_ptr<Node2d>> dp_map_;
};

}  // namespace planning
}  // namespace autoagric