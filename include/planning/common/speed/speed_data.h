#pragma once

#include <string>
#include <vector>

#include "autoagric/common/pnc_point.pb.h"

namespace autoagric {
namespace planning {

class SpeedData : public std::vector<common::SpeedPoint> {
 public:
  SpeedData() = default;

  virtual ~SpeedData() = default;

  explicit SpeedData(std::vector<common::SpeedPoint> speed_points);

  void AppendSpeedPoint(const double s, const double time, const double v,
                        const double a, const double da);

  bool EvaluateByTime(const double time,
                      common::SpeedPoint* const speed_point) const;
  // Assuming spatial traversed distance is monotonous, which is the case for
  // current usage on city driving scenario
  bool EvaluateByS(const double s, common::SpeedPoint* const speed_point) const;

  double TotalTime() const;

  double TotalLength() const;

  virtual std::string DebugString() const;
}

}  // namespace planning
}  // namespace autoagric