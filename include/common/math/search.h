#pragma once

#include <functional>

namespace autoagric {
namespace common {
namespace math {

double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol = 1e-6);
}
}  // namespace control
}  // namespace autoagric