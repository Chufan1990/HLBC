#include "planning/common/speed/st_boundary.h"

#include "common/macro.h"
#include "common/math/math_utils.h"
#include "planning/common/planning_gflags.h"

namespace autoagric {
namespace planning {
using common::math::LineSegment2d;
using common::math::Vec2d;

STBoundary::STBoundary(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs,
    bool is_accurate_boundary) {
  ACHECK(IsValid(point_pairs)) << "The input point_pairs are NOT valid";

  std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);
  if (!is_accurate_boundary) {
    RemoveRedundentPoints(&reduced_pairs);
  }

  for (const auto& item : reduced_pairs) {
    const double t = item.first.t();
    lower_points_.emplace_back(item.first.s(), t);
    upper_points_.emplace_back(item.second.s(), t);
  }

  for (const auto& point : lower_points_) {
    points_.emplace_back(point.t(), point.s());
  }

  for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); ++rit) {
    points_.emplace_back(rit->t(), rit->s());
  }

  BuildFromPoints();

  for (const auto& point : lower_points_) {
    min_s_ = std::fmin(min_s_, point.s());
  }

  for (const auto& point : upper_points_) {
    max_s_ = std::fmax(max_s_, point.s());
  }

  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();

  obstack_road_right_ending_t_ = std::numeric_limits<double>::lowest();
}

STBoundary STBoundary::CreateInstance(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  for (size_t i = 0; i < lower_points.size(); i++) {
    point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }

  return STBoundary(point_pairs, true);
}

std::string STBoundary::TypeName(BoundaryType type) {
  if (type == BoundaryType::FOLLOW) {
    return "FOLLOW";
  } else if (type == BoundaryType::KEEP_CLEAR) {
    return "KEEP_CLEAR";
  } else if (type == BoundaryType::OVERTAKE) {
    return "OVERTAKE";
  } else if (type == BoundaryType::STOP) {
    return "STOP";
  } else if (type == BoundaryType::YIELD) {
    return "YIELD";
  } else if (type == BoundaryType::UNKNOWN) {
    return "UNKNOWN";
  }
  AWARN("Unknown boundary type " << static_cast<int>(type)
                                 << ", treated as UNKNOWN");
  return "UNKNOWN";
}

bool STBoundary::GetUnblockSRange(const double curr_time, double* s_upper,
                                  double* s_lower) const {
  CHECK_NOTNULL(s_upper);
  CHECK_NOTNULL(s_lower);

  *s_upper = FLAGS_speed_lon_decision_horizon;
  *s_lower = 0.0;
  if (curr_time < min_t_ || curr_time > max_t_) {
    return true;
  }

  size_t left = 0;
  size_t right = 0;

  if (!GetIndexRange(lower_points_, curr_time, &left, &right)) {
    AERROR("Fail to get index range");
    return false;
  }

  if (curr_time > upper_points_[right].t()) {
    return true;
  }

  const double r =
      left == right ? 0.0
                    : (curr_time - upper_points_[left].t()) /
                          (upper_points_[right].t() - upper_points_[left].t());

  double upper_cross_s =
      upper_points_[left].s() +
      r * (upper_points_[right].s() - upper_points_[left].s());

  double lower_cross_s =
      lower_points_[left].s() +
      r * (lower_points_[right].s() - lower_points_[left].s());

  if (boundary_type_ == BoundaryType::STOP ||
      boundary_type_ == BoundaryType::YIELD ||
      BoundaryType == BoundaryType::FOLLOW) {
    *s_upper = lower_cross_s;
  } else if (boundary_type_ == BoundaryType::OVERTAKE) {
    *s_lower = std::fmax(*s_lower, upper_cross_s);
  } else {
        ADEBUG("boundary_type is not supported. boundary_type: "
           << static_cast<int>(boundary_type_);
           return false;
  }
  return true;
}

}  // namespace planning
}  // namespace autoagric