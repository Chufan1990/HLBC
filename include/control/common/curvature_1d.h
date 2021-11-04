#pragma once

#include <Eigen/Core>
#include <functional>
#include <memory>
#include <unsupported/Eigen/Splines>
#include <utility>
#include <vector>

namespace autoagric {
namespace control {

class Curvature1DTest;

class Curvature1D {
  friend class Curvature1DTest;

 public:
  typedef std::vector<std::pair<double, double>> DataType;

  Curvature1D() = default;

  bool Init(const DataType& xy);

  double Curvature(double x) const;

 private:
  double ScaledValue(double x) const;

  //   double Derivative1stOrder(double x) const;

  //   double Derivative2ndOrder(double x) const;

  double Derivatives(const double x, const int max_order,
                     const int order) const;

  Eigen::RowVectorXd ScaledValues(const Eigen::VectorXd& x_vec) const;

  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_start_ = 0.0;
  double y_end_ = 0.0;

  std::unique_ptr<Eigen::Spline<double, 1>> spline_;

  double scale_;
};

}  // namespace control
}  // namespace autoagric