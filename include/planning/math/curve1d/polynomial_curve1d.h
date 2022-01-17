#pragma once

#include "planning/math/curve1d/curve1d.h"

namespace autoagric {
namespace planning {
class PolynomialCurve1d : public Curve1d {
 public:
  PolynomialCurve1d() = default;
  virtual ~PolynomialCurve1d() = default;

  virtual double Coef(const size_t order) const = 0;

  virtual size_t Order() const = 0;

 protected:
  double param_ = 0.0;
};
}  // namespace planning
}  // namespace autoagric