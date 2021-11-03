#pragma once

#include <utility>
#include <vector>

#include "hlbc/proto/math/cos_theta_smoother_config.pb.h"

namespace autoagric {
namespace planning {
class CosThetaSmoother {
 public:
  explicit CosThetaSmoother(const CosThetaSmootherConfig& config);

  bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,
             const std::vector<double>& bounds, std::vector<double>* opt_x,
             std::vector<double>* opt_y);

 private:
  CosThetaSmootherConfig config_;
};

}  // namespace planning
}  // namespace autoagric