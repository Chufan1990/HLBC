#pragma once

#include <Eigen/Core>
#include <utility>
#include <vector>

#include "autoagric/planning/planner_open_space_config.pb.h"
#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/speed/speed_data.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace autoagric {
namespace planning {

class IterativeAnchoringSmoother {
 public:
  IterativeAnchoringSmoother(
      const PlannerOpenSpaceConfig& planner_open_space_config);

  ~IterativeAnchoringSmoother() = default;

  bool Smooth(const Eigen::MatrixXd& xWS, const double init_a,
              const double init_v,
              const std::vector<std::vector<common::math::Vec2d>>&
                  obstacls_vertices_vec,
              DiscretizedTrajectory* discretized_trajectory);

 private:
  void AdjustStartEndHeading(const Eigen::MatrixXd& xWS,
                             std::vector<std::pair<double, double>>* point2d);

  bool ReAnchoring(const std::vector<size_t>& colliding_point_index,
                   DiscretizedPath& path_points);

  bool GenerateInitialBounds(const DiscretizedPath& path_points,
                             std::vector<double>* initial_bounds);

  bool SmoothPath(const DiscretizedPath& raw_path_points,
                  const std::vector<double>& bounds,
                  DiscretizedPath* smoothed_path_points);

  bool CheckCollisionAvoidance(const DiscretizedPath& path_points,
                               std::vector<size_t>* colliding_point_index);

  void AdjustPathBounds(const std ::vector<size_t>& colliding_point_index,
                        std::vector<double>* bounds);

  bool SetPathProfile(const std::vector<std::pair<double, double>>& point2d,
                      DiscretizedPath* raw_path_points);

  bool CheckGear(const Eigen::MatrixXd& xWS);

  bool SmoothSpeed(const double init_a, const double init_v,
                   const double path_length, SpeedData* smoothed_speeds);

  bool CombinePathAndSpeed(const DiscretizedPath& path_points,
                           const SpeedData& speed_points,
                           DiscretizedTrajectory* discretized_trajectory);

  void AdjustPathAndSpeedByGear(DiscretizedTrajectory* discretized_trajectory);

  bool GenerateStopProfileFromPolynomial(const double init_acc,
                                         const double init_speed,
                                         const double stop_distane,
                                         SpeedData* smoothed_speeds);

  bool IsValidPolynomialProfile(const QuinticPolynomialCurve1d& curve);

  double CalcHeadings(const DiscretizedPath& path_points, const size_t index);

 private:
  double ego_length_ = 0.0;
  double ego_width_ = 0.0;
  double center_shift_distance_ = 0.0;
  double longitudinal_safety_margin_ = 0.0;
  double lateral_safety_margin_ = 0.0;

  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec_;

  std::vector<size_t> input_colliding_point_index_;

  bool enforce_initial_kappa_ = true;

  bool gear_ = false;

  PlannerOpenSpaceConfig planner_open_space_config_;
};

}  // namespace planning
}  // namespace  autoagric
