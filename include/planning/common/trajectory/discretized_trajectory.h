#pragma once

#include <vector>

#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/planning/planning.pb.h"
#include "common/macro.h"
#include "common/math/vec2d.h"

namespace autoagric {
namespace planning {

class DiscretizedTrajectory : public std::vector<common::TrajectoryPoint> {
 public:
  DiscretizedTrajectory() = default;

  /**
   * @brief construct a DiscretizedTrajectory instance by protobuf message
   */
  explicit DiscretizedTrajectory(const ADCTrajectory& trajectory);

  explicit DiscretizedTrajectory(
      const std::vector<common::TrajectoryPoint>& trajectory_points);

  void SetTrajectoryPoints(
      const std::vector<common::TrajectoryPoint>& trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  virtual common::TrajectoryPoint StartPoint() const;

  virtual double GetTemporalLength() const;

  virtual double GetSpatialLength() const;

  virtual common::TrajectoryPoint Evaluate(const double relative_time) const;

  virtual size_t QueryLowerBoundPoint(const double relative_time,
                                      const double epsilon = 1e-5) const;

  virtual size_t QueryNearestPoint(const common::math::Vec2d& position) const;

  size_t QueryNearestPointWithBuffer(const common::math::Vec2d& position,
                                     const double buffer) const;

  virtual void AppendTrajectoryPoint(
      const common::TrajectoryPoint& trajectory_point);

  void PrependTrajectoryPoints(
      const std::vector<common::TrajectoryPoint>& trajectory_points) {
    if (!empty() && trajectory_points.size() > 1) {
      ACHECK(trajectory_points.back().relative_time() <
             front().relative_time());
    }

    insert(begin(), trajectory_points.begin(), trajectory_points.end());
  }

  const common::TrajectoryPoint& TrajectoryPointAt(const size_t index) const;

  size_t NumOfPoints() const;

  virtual void Clear();
};

inline size_t DiscretizedTrajectory::NumOfPoints() const { return size(); }

inline void DiscretizedTrajectory::Clear() { clear(); }

}  // namespace planning
}  // namespace autoagric