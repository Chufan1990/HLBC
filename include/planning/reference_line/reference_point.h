/**
 * @file reference_point.h
 */
#pragma once

#include <string>

#include "hlbc/proto/pnc_point.pb.h"

namespace autoagric {
namespace planning {

class ReferencePoint {
 public:
  ReferencePoint() = default;

  //   ReferencePoint(const MapPathPoint& map_path_point, const double kappa,
  //              const double dkappa);

  common::PathPoint ToPathPoint(double s) const;

  double kappa() const;
  double dkappa() const;

  std::string DebugString() const;

  static void RemoveDuplicates(std::vector<ReferencePoint>* points);

 private:
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};

}  // namespace planning
}  // namespace autoagric