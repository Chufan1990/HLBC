#pragma once
#include <Eigen/Core>
#include <memory>

#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/planning/open_space_trajectory_generator_config.pb.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"
#include "planning/open_space/trajectory_smoother/iterative_anchoring_smoother.h"

namespace autoagric {
namespace planning {

class OpenSpaceTrajectoryGenerator {
 public:
  OpenSpaceTrajectoryGenerator(
      const OpenSpaceTrajectoryGeneratorConfig& config);

  common::Status Plan(
      const std::vector<common::TrajectoryPoint>& stitching_trajectory,
      const std::vector<double>& cur_pose, const std::vector<double>& end_pose,
      const std::vector<double>& XYbounds, const double rotate_angle,
      const common::math::Vec2d& translate_origin,
      const std::vector<std::vector<common::math::Vec2d>>&
          obstacles_vertices_vec);

  void GetOptimizedTrajectory(DiscretizedTrajectory* optimized_trajectory) {
    optimized_trajectory->clear();
    *optimized_trajectory = optimized_trajectory_;
  }

 private:
  void PathPointNormalizing(const double rotate_angle,
                            const common::math::Vec2d& translate_origin,
                            double* x, double* y, double* phi);

  void PathPointDeNormalizing(const double rotate_angle,
                              const common::math::Vec2d& tranlate_origin,
                              double* x, double* y, double* phi);

  void LoadHybridAstarResultInEigen(HybridAStarResult* result,
                                    Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS);

  bool GenerateDecoupledTraj(
      const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
      const std::vector<std::vector<common::math::Vec2d>>&
          obstacles_vertices_vec,
      Eigen::MatrixXd* state_result_dc, Eigen::MatrixXd* control_result_dc,
      Eigen::MatrixXd* time_result_dc);

  void LoadResult(const DiscretizedTrajectory& discretized_trajectory,
                  Eigen::MatrixXd* state_result_dc,
                  Eigen::MatrixXd* control_result_dc,
                  Eigen::MatrixXd* time_result_dc);

  bool GenerateIterativeAnchoringTraj(
      const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
      const std::vector<std::vector<common::math::Vec2d>>&
          obstacles_vertices_vec);

  void CombineTrajectories(
      const std::vector<Eigen::MatrixXd>& xWS_vec,
      const std::vector<Eigen::MatrixXd>& uWS_vec,
      const std::vector<Eigen::MatrixXd>& state_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& control_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& time_result_ds_vec,
      Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS,
      Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
      Eigen::MatrixXd* time_result_ds);

  void LoadTrajectory(const Eigen::MatrixXd& state_result_ds,
                      const Eigen::MatrixXd& control_result_ds,
                      const Eigen::MatrixXd& time_result_ds);

  std::unique_ptr<HybridAStar> warm_start_;

  std::unique_ptr<IterativeAnchoringSmoother> iterative_anchoring_smoother_;

  OpenSpaceTrajectoryGeneratorConfig config_;

  std::shared_ptr<DependencyInjector> injector_;

  DiscretizedTrajectory optimized_trajectory_;
};
}  // namespace planning
}  // namespace autoagric