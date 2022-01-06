#include "planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

#include <algorithm>
#include <chrono>
#include <limits>

#include "common/math/box2d.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "planning/common/planning_gflags.h"

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
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ADEBUG("speed profile total time: " << diff.count() * 1000.0 << " ms.");
  return true;
}

  bool HybridAStar::GenerateSpeedAcceleration(HybridAStarResult* result){}
  bool HybridAStar::GenerateSCurveSpeedAcceleration(HybridAStarResult* result){}

}  // namespace planning
}  // namespace autoagric