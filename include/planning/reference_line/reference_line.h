/**
 * @file reference_line.h
 */
#pragma once

#include <vector>

#include "planning/reference_line/reference_line.h"
#include "planning/reference_line/reference_point.h"

namespace autoagric {
namespace planning {

class ReferenceLine {
 public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;

  //   template <typename Iterator>
  //   ReferenceLine(const Iterator begin, const Iterator end)
  //       : reference_points_(begin, end),
  //         map_path_(std::move(std::vector<hdmap::MapPathPoint>(begin, end)))
  //         {}

  explicit ReferenceLine(const std::vector<ReferencePoint>* reference_points);
  //   explicit ReferenceLine(const hdmap::Path& hdmap_path);
};

}  // namespace planning
}  // namespace autoagric