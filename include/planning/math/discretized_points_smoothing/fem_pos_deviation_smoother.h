#pragma once

#include "autoagric/planning/math/fem_pos_deviation_smoother_config.pb.h"

#include <vector>
#include <utility>

namespace autoagric {
namespace planning {

class FemPosDeviationSmoother {
 public:
  explicit FemPosDeviationSmoother(const FemPosDeviationSmootherConfig& config);

  bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,
             const std::vector<double>& bounds, std::vector<double>* opt_x,
             std::vector<double>* opt_y);

  bool QpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                  const std::vector<double>& bounds, std::vector<double>* opt_x,
                  std::vector<double>* opt_y);

  bool NlpWithIpopt(const std::vector<std::pair<double, double>>& raw_point2,
                    const std::vector<double>& bounds,
                    std::vector<double>* opt_x, std::vector<double>* opt_y);

  bool SqpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2,
                   const std::vector<double>& bounds,
                   std::vector<double>* opt_x, std::vector<double>* opt_y);

 private:
  FemPosDeviationSmootherConfig config_;
};

}  // namespace planning
}  // namespace autoagric