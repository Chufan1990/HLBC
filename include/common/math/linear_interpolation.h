#pragma once

#include <ros/console.h>

#include <cmath>

#include "common/macro.h"
#include "autoagric/common/pnc_point.pb.h"

namespace autoagric {
namespace common {
namespace math {

template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  if (std::fabs(t1 - t0) <= 1e-6) {
    AERROR("linear_interpolation.h, lerp",
           "input time difference is too small\n");
    return x0;
  }

  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t);

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s);

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0,
                                                    const TrajectoryPoint &tp1,
                                                    const double t);
}  // namespace math
}  // namespace common
}  // namespace autoagric