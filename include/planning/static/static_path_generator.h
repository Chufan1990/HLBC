#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "autoagric/planning/static_path_config.pb.h"
#include "common/util/csv_reader.h"
#include "planning/math/discretized_points_smoothing/cos_theta_smoother.h"

namespace autoagric {
namespace planning {

class StaticPathGenerator {
 public:
  StaticPathGenerator() = default;

  StaticPathGenerator(const std::string& file_path);

  bool Init(const planning::StaticPathConfig& config);

  bool Proc();

  common::util::StaticPathResult GenerateLocalProfile(const double x,
                                                      const double y);

  const common::util::StaticPathResult& Path() const { return path_; }

 private:
  bool GetTemporalProfile(common::util::StaticPathResult* result);

  bool GenerateSpeedAcceleration(common::util::StaticPathResult* result);

  bool GenerateSCurveSpeedAcceleration(common::util::StaticPathResult* result);

  bool TrajectoryPartition(
      const common::util::StaticPathResult& result,
      std::vector<common::util::StaticPathResult>* paritioned_results);

  bool GenerateSmoothPathProfle(common::util::StaticPathResult* result);

  std::pair<int, int> QueryNearestPointByPoistion(const double x,
                                                  const double y,
                                                  const size_t index) const;

  bool CosThetaSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  size_t current_start_index_;

  size_t path_length_;

  std::unique_ptr<common::util::CSVReader> csv_reader_;

  std::string file_path_;

  common::util::StaticPathResult path_;

  planning::StaticPathConfig config_;

  std::unique_ptr<planning::CosThetaSmoother> cos_theta_smoother_;

  double zero_x_;

  double zero_y_;
};
}  // namespace planning
}  // namespace autoagric
