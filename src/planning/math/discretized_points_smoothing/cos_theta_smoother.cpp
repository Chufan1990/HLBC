#include "planning/math/discretized_points_smoothing/cos_theta_smoother.h"

#include <cppad/cppad.hpp>
#include <memory>

#include "planning/math/discretized_points_smoothing/cos_theta_ipopt_interface.h"

namespace autoagric {
namespace planning {

CosThetaSmoother::CosThetaSmoother(const CosThetaSmootherConfig& config)
    : config_(config) {}

bool CosThetaSmoother::Solve(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  const double weight_cos_included_angle = config_.weight_cos_included_angle();
  const double weight_anchor_points = config_.weight_anchor_points();
  const double weight_length = config_.weight_length();
  const size_t print_level = config_.print_level();
  const size_t max_num_of_iterations = config_.max_num_of_iterations();
  const size_t acceptable_num_of_iterations =
      config_.acceptable_num_of_iterations();
  const double tol = config_.tol();
  const double acceptable_tol = config_.acceptable_tol();
  const bool use_automatic_differentiation =
      config_.ipopt_use_automatic_differentiation();

  std::unique_ptr<CosThetaIpoptInterface>* smoother =
      std::move(std::unique_ptr<CosThetaIpoptInterface>(
          new CosThetaIpoptInterface(raw_point2d, bounds)));

  smoother->set_weight_cos_included_angle(weight_cos_included_angle);
  smoother->set_weight_anchor_points(weight_anchor_points);
  smoother->set_weight_length(weight_length);

  int n = m = raw_point2d.size() << 1;

  Dvector x(n);
  Dvector x_l(n), x_u(n);
  Dvector g_l(m), g_u(m);

  CHECK(smoother->get_starting_point(n, x));
  CHECK(smoother->get_bounds_info(n, x_l, x_u, m, g_l, g_u));


  option += "Integer print_level " + std::to_string(print_level) + "\n";
  option += "Integer max_iter " + std::to_string(max_num_of_iterations) + "\n";
  option += "Integer print_level " + std::to_string(print_level) + "\n";
  option += "Integer print_level " + std::to_string(print_level) + "\n";
  option += "Integer print_level " + std::to_string(print_level) + "\n";
  option += "Integer print_level " + std::to_string(print_level) + "\n";


  return true;
}

}  // namespace planning

}  // namespace autoagric
