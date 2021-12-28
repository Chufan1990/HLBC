#include "planning/open_space/coarse_trajectory_generator/reeds_shepp_path.h"

#include "common/macro.h"
#include "common/math/math_utils.h"
#include "planning/common/planning_gflags.h"

namespace autoagric {
namespace planning {

ReedsShepp::ReedsShepp(const common::VehicleParam& vehicle_param,
                       const PlannerOpenSpaceConfig& open_space_conf)
    : vehicle_param_(vehicle_param),
      planner_open_space_config_(opens_space_conf) {
  max_kappa_ =
      std::tan(vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio() /
               vehicle_param_.wheel_base());

  AINFO_IF(FLAGS_enable_parallel_hybrid_a, "parallel ReedsShepp");
}

std::pair<double, double> ReedsShepp::calc_tau_omega(const double u,
                                                     const double v,
                                                     const double xi,
                                                     const double eta,
                                                     const double phi) {
  double delta = common::math::NormalizeAngle(u - v);
  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.0;

  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  double tau = 0.0;
  if (t2 < 0) {
    tau = common::math::NormalizeAngle(t1 + M_PI);
  } else {
    tau = common::math::NormalizeAngle(t1);
  }

  double omega = common::math::NormalizeAngle(tau - u + v - phi);
  return std::make_pair(tau, omega);
}

bool ReedsShepp::ShortestRSP(const std::shared_ptr<Node3d> start_node,
                             const std::shared_ptr<Node3d> end_node,
                             std::shared_ptr<ReedsSheppPath> optimal_path) {
  std::vector<ReedsSheppPath> all_possible_paths;
  if (!GenerateRSPs(start_node, end_node, &all_possible_paths)) {
    ADEBUG("Fail to generate different combination of Reeds Shepp paths");
    return false;
  }

  double optimal_path_length = std::numeric_limits<double>::infinity();
  size_t optimal_path_index = 0;
  size_t paths_size = all_possible_paths.size();
  for (int i = 0; i < paths_size; i++) {
    if (all_possible_paths.at(i).total_length > 0 &&
        all_possible_paths.at(i).total_length < optimal_path_length) {
      optimal_path_index = i;
      optimal_path_length = all_possible_paths.at(i).total_length;
    }
  }

  auto& optimal_path_candidate = all_possible_paths[optimal_path_index];

  if (!GenerateLocalConfiguration(start_node, end_node,
                                  &optimal_path_candidate)) {
    ADEBUG("Fail to generate local configuration(x, y, phi) in SetRSP");
    return false;
  }

  if (std::abs(optimal_path_candidate.x.back() - end_node->GetX()) > 1e-3 ||
      std::abs(optimal_path_candidate.y.back() - end_node->GetY()) > 1e-3 ||
      std::abs(optimal_path_candidate.phi.back() - end_node->GetPhi()) > 1e-3) {
    ADEBUG("RSP end position not right");
    for (size_t i = 0; i < optimal_path_candidate.segs_types.size(); i++) {
      ADEBUG("types are " << optimal_path_candidate.segs_types[i]);
    }
    ADEBUG("x, y, phi are: " << optimal_path_candidate.x.back() << ", "
                             << optimal_path_candidate.y.back() << ", "
                             << optimal_path_candidate.phi.back());
    ADEBUG("end x, y, phi are: " << end_node->GetX() << ", " << end_node->GetY()
                                 << ", " << end_node->GetPhi());
    return false;
  }

  optimal_path->x = std::move(optimal_path_candidate.x);
  optimal_path->y = std::move(optimal_path_candidate.y);
  optimal_path->phi = std::move(optimal_path_candidate.phi);
  optimal_path->gear = std::move(optimal_path_candidate.gear);
  optimal_path->total_length = std::move(optimal_path_candidate.total_length);
  optimal_path->segs_lengths = std::move(optimal_path_candidate.segs_lengths);
  optimal_path->segs_types = std::move(optimal_path_candidate.segs_types);
}

}  // namespace planning
}  // namespace autoagric