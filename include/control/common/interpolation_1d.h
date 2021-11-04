#pragma once

#include <Eigen/Core>
#include <memory>
#include <unsupported/Eigen/Splines>
#include <utility>
#include <vector>

namespace autoagric {
namespace control {

class Interpolation1D {
 public:
  typedef std::vector<std::pair<double, double>> DataType;

  Interpolation1D() = default;

  bool Init(const DataType& xy);

  double Interpolate(double x) const;

 private:
  double ScaledValue(double x) const;

  Eigen::RowVectorXd ScaledValues(const Eigen::VectorXd& x_vec) const;

  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_start_ = 0.0;
  double y_end_ = 0.0;

  std::unique_ptr<Eigen::Spline<double, 1>> spline_;
};

}  // namespace control
}  // namespace autoagric