#pragma once

#include <vector>

#include "common/math/vec2d.h"
#include "planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

namespace autoagric {
namespace planning {

class HybridAObstacleContainer {
 public:
  HybridAObstacleContainer() = default;
  void AddVirtualObstacle(double* obstacle_x, double* obstacle_y,
                          int vertice_num) {
    std::vector<common::math::Vec2d> obstacle_vertices;
    for (int i = 0; i < vertice_num; i++) {
      common::math::Vec2d vertice(obstacle_x[i], obstacle_y[i]);
      obstacle_vertices.emplace_back(vertice);
    }
    obstacles_list_.emplace_back(obstacle_vertices);
  }
  const std::vector<std::vector<common::math::Vec2d>>&
  GetObstaclesVerticesVec() {
    return obstacles_list_;
  }

 private:
  std::vector<std::vector<common::math::Vec2d>> obstacles_list_;
};

class HybridAResultContainer {
 public:
  HybridAResultContainer() = default;
  void LoadResult() {
    x_ = std::move(result_.x);
    y_ = std::move(result_.y);
    phi_ = std::move(result_.phi);
    v_ = std::move(result_.v);
    a_ = std::move(result_.a);
    steer_ = std::move(result_.steer);
  }
  std::vector<double>* GetX() { return &x_; }
  std::vector<double>* GetY() { return &y_; }
  std::vector<double>* GetPhi() { return &phi_; }
  std::vector<double>* GetV() { return &v_; }
  std::vector<double>* GetA() { return &a_; }
  std::vector<double>* GetSteer() { return &steer_; }
  HybridAStartResult* PrepareResult() { return &result_; }

 private:
  HybridAStartResult result_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> phi_;
  std::vector<double> v_;
  std::vector<double> a_;
  std::vector<double> steer_;
};

}  // namespace planning
}  // namespace  autoagric
