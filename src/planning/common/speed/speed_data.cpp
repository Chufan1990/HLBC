#include "planning/common/speed/speed_data.h"

#include <algorithm>
#include <mutex>
#include <utility>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "common/math/linear_interpolation.h"
#include "common/util/point_factory.h"
#include "common/util/string_util.h"
#include "common/util/util.h"
#include "planning/common/planning_gflags.h"

namespace autoagric {
namespace planning {
using common::SpeedPoint;

SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
    : std::vector<SpeedPoint>(std::move(speed_points)) {
  std::sort(begin(), end(), [](const SpeedPoint& p1, const SpeedPoint& p2) {
    return p1.t() < p2.t();
  });
}

void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a = 0.0,
                                 const double da = 0.0) {
  static std::mutex mutex_speedpoint;
  UNIQUE_LOCK_MULTITHREAD(mutex_speedpoint);

  if (!empty()) {
    ACHECK(back().t() < time);
  }
  push_back(common::util::PointFactory::ToSpeedPoint(s, time, v, a, da));
}

bool SpeedData::EvaluateByTime(const double t,
                               SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().t() < t + 1e-6 && t - 1e-6 < back().t())) {
    return false;
  }

  auto comp = [](const common::SpeedPoint& sp, const double t) {
    return sp.t() < t;
  };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.t();
    double t1 = p1.t();

    speed_point->Clear();
    speed_point->set_s(common::math::lerp(p0.s(), t0, p1.s(), t1, t));
    speed_point->set_t(t);
    speed_point->set_v(common::math::lerp(p0.v(), t0, p1.v(), t1, t));
    speed_point->set_a(common::math::lerp(p0.a(), t0, p1.a(), t1, t));
    speed_point->set_da(common::math::lerp(p0.da(), t0, p1.da(), t1, t));
  }
  return true;
}

bool SpeedData::EvaluateByS(const double s,
                            common::SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().s() < s + 1.0e-6 && s - 1.0e-6 < back().s())) {
    return false;
  }

  auto comp = [](const common::SpeedPoint& sp, const double s) {
    return sp.s() < s;
  };

  auto it_lower = std::lower_bound(begin(), end(), s, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double s0 = p0.s();
    double s1 = p1.s();

    speed_point->Clear();
    speed_point->set_s(s);
    speed_point->set_t(common::math::lerp(p0.t(), s0, p1.t(), s1, s));
    speed_point->set_v(common::math::lerp(p0.v(), s0, p1.v(), s1, s));
    speed_point->set_a(common::math::lerp(p0.a(), s0, p1.a(), s1, s));
    speed_point->set_da(common::math::lerp(p0.da(), s0, p1.da(), s1, s));
  }
  return true;
}

double SpeedData::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t() - front().t();
}

double SpeedData::TotalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

std::string SpeedData::DebugString() const {
  const auto limit = std::min(
      size(), static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  return absl::StrCat(
      "[\n",
      absl::StrJoin(begin(), begin() + limit, ",\n",
                    common::util::DebugStringFormatter()),
      "]\n");
}

}  // namespace planning
}  // namespace autoagric