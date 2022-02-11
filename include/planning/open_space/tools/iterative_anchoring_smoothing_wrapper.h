#pragma once

#include <geometry_msgs/Point.h>
#include <nav_msgs/GridCells.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <vector>

#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/planning/open_space_task_config.pb.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "common/math/vec2d.h"
#include "common/util/trajectory_visualizer.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

namespace autoagric {
namespace planning {

class IterativeAnchoringSmoothingWrapper {
 public:
  IterativeAnchoringSmoothingWrapper() = default;

  IterativeAnchoringSmoothingWrapper(ros::NodeHandle& nh);

  virtual ~IterativeAnchoringSmoothingWrapper();

  bool Init();

  bool Proc();

 private:
  void GetRoiBoundary(const double sx, const double sy, const double sphi,
                      const double ex, const double ey, const double ephi,
                      std::vector<common::math::Vec2d>& boundary_points,
                      std::vector<double>* XYbounds);

  bool GetVirtualParkingLot(
      const double sx, const double sy, const double stheta,
      std::vector<std::vector<common::math::Vec2d>>* obstacles_vertices_vec,
      std::vector<double>* end_pose);

  std::vector<common::math::Vec2d> CenterToRectangle(
      const double cx, const double cy, const double cphi, const double left,
      const double right, const double front, const double rear);

  void PathPointNormalizing(const double rotate_angle,
                            const common::math::Vec2d& translate_origin,
                            double* x, double* y, double* phi);

  void PathPointDeNormalizing(const double rotate_angle,
                              const common::math::Vec2d& translate_origin,
                              double* x, double* y, double* phi);

  void LoadHybridAstarResultInEigen(HybridAStarResult* result,
                                    Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS);

  void LoadResult(const DiscretizedTrajectory& discretized_trajectory,
                  Eigen::MatrixXd* state_result_dc,
                  Eigen::MatrixXd* control_result_dc,
                  Eigen::MatrixXd* time_result_dc);

  void CombineTrajectories(
      const std::vector<Eigen::MatrixXd>& xWS_vec,
      const std::vector<Eigen::MatrixXd>& uWS_vec,
      const std::vector<Eigen::MatrixXd>& state_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& control_result_ds_vec,
      const std::vector<Eigen::MatrixXd>& time_result_ds_vec,
      Eigen::MatrixXd* xWS, Eigen::MatrixXd* uWS,
      Eigen::MatrixXd* state_result_ds, Eigen::MatrixXd* control_result_ds,
      Eigen::MatrixXd* time_result_ds);

  DiscretizedTrajectory LoadTrajectory(const Eigen::MatrixXd& state_result,
                                       const Eigen::MatrixXd& control_result,
                                       const Eigen::MatrixXd& time_result);

  void Visualize(const ros::TimerEvent& e);

 private:
  ros::NodeHandle nh_;

  std::unique_ptr<ros::AsyncSpinner> spinner_;

  DiscretizedTrajectory optimized_trajectory_;

  std::shared_ptr<DependencyInjector> injector_;

  std::unique_ptr<common::util::TrajectoryVisualizer> warm_start_visualizer_;

  std::unique_ptr<common::util::TrajectoryVisualizer>
      optimized_trajectory_visualizer_;

  std::unique_ptr<common::util::TrajectoryVisualizer> obstacle_visualizer_;

  std::unordered_map<std::string, visualization_msgs::MarkerArray>
      warm_start_markers_;

  std::unordered_map<std::string, visualization_msgs::MarkerArray>
      optimized_trajectory_markers_;

  std::unordered_map<std::string, visualization_msgs::MarkerArray>
      obstacle_markers_;

  std::unique_ptr<ros::Timer> trajectory_marker_writer_;

  std::unique_ptr<ros::Publisher> dp_map_writer_;

  IterativeAnchoringSmoothingWrapperConfig config_;

  std::atomic<bool> trajectory_updated_{false};
  std::atomic<bool> obstacles_updated_{false};
  std::atomic<bool> dp_map_updated_{false};

  nav_msgs::GridCells dp_map_;

  double start_point_x_ = 10.0;

  double start_point_y_ = 10.0;

  double start_point_phi_ = 0.0;

  bool parallel_parking_ = false;
};

}  // namespace planning
}  // namespace autoagric