#include "common/math/math_utils.h"

#include "Eigen/Core"
#include <cmath>
#include <limits>
#include <utility>

namespace autoagric {
namespace common {
namespace math {

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += (2.0 * M_PI);
  return a - M_PI;
}

}  // namespace math
}  // namespace common
}  // namespace autoagric