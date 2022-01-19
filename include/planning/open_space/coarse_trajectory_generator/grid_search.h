#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "autoagric/planning/planner_open_space_config.pb.h"
#include "common/math/line_segment2d.h"

namespace autoagric {
namespace planning {


class Node2d {
 public:
  Node2d(const double x, const double y, const double xy_resolution,
         const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    grid_x_ = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    grid_y_ = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }
  Node2d(const int grid_x, const int grid_y,
         const std::vector<double>& XYbounds) {
    grid_x_ = grid_x;
    grid_y_ = grid_y;
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }
  void SetPathCost(const double path_cost) {
    path_cost_ = path_cost;
    cost_ = path_cost_ + heuristic_;
  }
  void SetHeuristic(const double heuristic) {
    heuristic_ = heuristic;
    cost_ = path_cost_ + heuristic_;
  }
  void SetCost(const double cost) { cost_ = cost; }
  void SetPreNode(std::shared_ptr<Node2d> pre_node) { pre_node_ = pre_node; }
  double GetGridX() const { return grid_x_; }
  double GetGridY() const { return grid_y_; }
  double GetPathCost() const { return path_cost_; }
  double GetHeuCost() const { return heuristic_; }
  double GetCost() const { return cost_; }
  const std::string& GetIndex() const { return index_; }
  std::shared_ptr<Node2d> GetPreNode() const { return pre_node_; }
  static std::string CalcIndex(const double x, const double y,
                               const double xy_resolution,
                               const std::vector<double>& XYbounds) {
    // XYbounds with xmin, xmax, ymin, ymax
    int grid_x = static_cast<int>((x - XYbounds[0]) / xy_resolution);
    int grid_y = static_cast<int>((y - XYbounds[2]) / xy_resolution);
    return ComputeStringIndex(grid_x, grid_y);
  }
  bool operator==(const Node2d& right) const {
    return right.GetIndex() == index_;
  }

 private:
  static std::string ComputeStringIndex(int x_grid, int y_grid) {
    return absl::StrCat(x_grid, "_", y_grid);
  }

 private:
  int grid_x_ = 0;
  int grid_y_ = 0;
  double path_cost_ = 0.0;
  double heuristic_ = 0.0;
  double cost_ = 0.0;
  std::string index_;
  std::shared_ptr<Node2d> pre_node_ = nullptr;
};

struct GridAStarResult {
  std::vector<double> x;
  std::vector<double> y;
  double path_cost = 0.0;
};

class GridSearch {
 public:
  explicit GridSearch(const PlannerOpenSpaceConfig& open_space_conf);
  virtual ~GridSearch() = default;
  bool GenerateAStarPath(
      const double sx, const double sy, const double ex, const double ey,
      const std::vector<double>& XYbounds,
      const std::vector<std::vector<common::math::LineSegment2d>>&
          obstacles_linesegments_vec,
      GridAStarResult* result);
  bool GenerateDpMap(
      const double ex, const double ey, const std::vector<double>& XYbounds,
      const std::vector<std::vector<common::math::LineSegment2d>>&
          obstacles_linesegments_vec);
  double CheckDpMap(const double sx, const double sy);

 private:
  double EuclidDistance(const double x1, const double y1, const double x2,
                        const double y2);
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      std::shared_ptr<Node2d> node);
  bool CheckConstraints(std::shared_ptr<Node2d> node);
  void LoadGridAStarResult(GridAStarResult* result);

 private:
  double xy_grid_resolution_ = 0.0;
  double node_radius_ = 0.0;
  std::vector<double> XYbounds_;
  double max_grid_x_ = 0.0;
  double max_grid_y_ = 0.0;
  std::shared_ptr<Node2d> start_node_;
  std::shared_ptr<Node2d> end_node_;
  std::shared_ptr<Node2d> final_node_;
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;

  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };
  std::unordered_map<std::string, std::shared_ptr<Node2d>> dp_map_;
};

}  // namespace planning
}  // namespace autoagric
