#include "control/common/interpolation_2d.h"

#include <cmath>

#include "common/macro.h"

namespace {

const double kDoubleEpsilon = 1e-6;

}

namespace autoagric {
namespace control {

bool Interpolation2D::Init(const DataType& xyz) {
  if (xyz.empty()) {
    AERROR(
           "empty input.");
    return false;
  }
  for (const auto& t : xyz)
    xyz_[std::get<0>(t)][std::get<1>(t)] = std::get<2>(t);

  return true;
}

double Interpolation2D::Interpolate(const KeyType& xy) const {
  double max_x = xyz_.rbegin()->first;
  double min_x = xyz_.begin()->first;
  if (xy.first >= max_x - kDoubleEpsilon)
    return InterpolateYz(xyz_.rbegin()->second, xy.second);
  if (xy.first <= min_x + kDoubleEpsilon)
    return InterpolateYz(xyz_.begin()->second, xy.second);

  auto iter_after = xyz_.lower_bound(xy.first);
  auto iter_before = iter_after;
  if (iter_before != xyz_.begin()) iter_before--;

  double x_before = iter_before->first;
  double z_before = InterpolateYz(iter_before->second, xy.second);
  double x_after = iter_after->first;
  double z_after = InterpolateYz(iter_after->second, xy.second);

  double x_diff_before = std::fabs(xy.first - x_before);
  double x_diff_after = std::fabs(xy.first - x_after);

  return InterpolateValue(z_before, x_diff_before, z_after, x_diff_after);
}

double Interpolation2D::InterpolateYz(const std::map<double, double>& yz_table,
                                      double y) const {
  if (yz_table.empty()) {
    ADEBUG_EVERY(
        1, 
        "Unable to interpolateYz because yz_table is empty.");
    return y;
  }

  double max_y = yz_table.rbegin()->first;
  double min_y = yz_table.begin()->first;
  if (y >= max_y - kDoubleEpsilon) return yz_table.rbegin()->second;
  if (y <= min_y + kDoubleEpsilon) return yz_table.begin()->second;

  auto iter_after = yz_table.lower_bound(y);
  auto iter_before = iter_after;

  if (iter_before != yz_table.begin()) iter_before--;

  double y_before = iter_before->first;
  double z_before = iter_before->second;
  double y_after = iter_after->first;
  double z_after = iter_after->second;

  double y_diff_before = std::fabs(y - y_before);
  double y_diff_after = std::fabs(y - y_after);

  return InterpolateValue(z_before, y_diff_before, z_after, y_diff_after);
}

double Interpolation2D::InterpolateValue(const double value_before,
                                         const double dist_before,
                                         const double value_after,
                                         const double dist_after) const {
  if (dist_before < kDoubleEpsilon) return value_before;
  if (dist_after < kDoubleEpsilon) return value_after;
  double value_gap = value_after - value_before;
  double value_buff = value_gap * dist_before / (dist_before + dist_after);
  return value_before + value_buff;
}

}  // namespace control
}  // namespace autoagric