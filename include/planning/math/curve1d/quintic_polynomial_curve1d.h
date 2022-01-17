#pragma once

#include <array>
#include <string>

#include "planning/math/curve1d/polynomial_curve1d.h"

namespace autoagric {
namespace planning {

class QuinticPolynomialCurve1d : public PolynomialCurve1d {
 public:
  QuinticPolynomialCurve1d() = default;

  QuinticPolynomialCurve1d(const std::array<double, 3>& start,
                           const std::array<double, 3>& end,
                           const double param);

  QuinticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                           const double x1, const double dx1, const double ddx1,
                           const double param);

  QuinticPolynomialCurve1d(const QuinticPolynomialCurve1d& other);

  void SetParam(const double x0, const double dx0, const double ddx0,
                const double x1, const double dx1, const double ddx1,
                const double param);

  void IntegratedFromQuarticCurve(const PolynomialCurve1d& other,
                                  const double init_value);

  virtual ~QuinticPolynomialCurve1d() = default;

  double Evaluate(const std::uint32_t order, const double p) const override;

  double ParamLength() const override { return param_; }

  std::string ToString() const override;

  double Coef(const size_t order) const override;

  size_t Order() const override { return 5; }

 protected:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double x1, const double dx1, const double ddx1,
                           const double param);

  std::array<double, 6> coef_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::array<double, 3> start_condition_ = {0.0, 0.0, 0.0};

  std::array<double, 3> end_condition_ = {0.0, 0.0, 0.0};
};
}  // namespace planning
}  // namespace autoagric
