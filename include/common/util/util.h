/**
 * @file util.h
 * @brief some util functions
 */

#pragma once

#include <cstdint>
#include <vector>

/**
 * @namespace autoagric::common::util
 * @namespace autoagric::common::util
 */
namespace autoagric {
namespace common {
namespace util {

/**
 * uniformly slice a segment [start, end] to num + 1 pieces
 * the result sliced will contain the n + 1 points that slices the provided
 * segment. `start` and `end` will be the first and last element in `sliced`.
 */
template <typename T>
void uniform_slice(const T start, const T end, uint32_t num,
                   std::vector<T>* sliced) {
  if (!sliced || num == 0) {
    return;
  }
  const T delta = (end - start) / num;
  sliced->resize(num + 1);

  T s = start;
  for (uint32_t i = 0; i < num; i++, s += delta) {
    sliced->at(i) = s;
  }
  sliced->at(num) = end;
}

}  // namespace util
}  // namespace common
}  // namespace autoagric