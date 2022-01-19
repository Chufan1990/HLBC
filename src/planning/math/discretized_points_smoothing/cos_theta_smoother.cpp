#include "planning/math/discretized_points_smoothing/cos_theta_smoother.h"

#include <cppad/cppad.hpp>
#include <memory>
#include <string>

#include "common/macro.h"
#include "glog/logging.h"
#include "planning/math/discretized_points_smoothing/cos_theta_ipopt_interface.h"

namespace autoagric {
namespace planning {

using Dvector = CosThetaIpoptInterface::Dvector;

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
  // const size_t acceptable_num_of_iterations =
  //     config_.acceptable_num_of_iterations();
  const double tol = config_.tol();
  const double acceptable_tol = config_.acceptable_tol();
  // const bool use_automatic_differentiation =
  //     config_.ipopt_use_automatic_differentiation();

  std::unique_ptr<CosThetaIpoptInterface> smoother =
      std::unique_ptr<CosThetaIpoptInterface>(
          new CosThetaIpoptInterface(raw_point2d, bounds));

  smoother->set_weight_cos_included_angle(weight_cos_included_angle);
  smoother->set_weight_anchor_points(weight_anchor_points);
  smoother->set_weight_length(weight_length);

  int n = raw_point2d.size() << 1;
  int m = raw_point2d.size() << 1;
  Dvector x(n);
  Dvector x_l(n), x_u(n);
  Dvector g_l(m), g_u(m);

  CHECK(smoother->get_starting_point(n, x));
  CHECK(smoother->get_bounds_info(n, x_l, x_u, m, g_l, g_u));

  // for (size_t i = 0; i < raw_point2d.size(); i++) {
  //   size_t index = i << 1;
  //   ADEBUG("Point " << i << " x: " << x_l[index] << " <= " << x[index]
  //                       << " <= " << x_u[index]);
  //   ADEBUG("Point " << i << " y: " << x_l[index + 1]
  //                       << " <= " << x[index + 1] << " <= " << x_u[index +
  //                       1]);
  // }

  // for (size_t i = 0; i < raw_point2d.size(); i++) {
  //   size_t index = i << 1;
  //   ADEBUG("",
  //          "New Point " << i << " x: " << g_l[index] << " to " <<
  //          g_u[index]);
  //   ADEBUG("New Point " << i << " y: " << g_l[index + 1] << " to "
  //                           << g_u[index + 1]);
  // }

  // options
  std::string options;
  // turn off any printing
  options += "Integer print_level " + std::to_string(print_level) + "\n";
  // maximum number of iterations
  options += "Integer max_iter " + std::to_string(max_num_of_iterations) + "\n";
  // approximate accuracy in first order necessary conditions;
  // see Mathematical Programming, Volume 106, Number 1,
  // Pages 25-57, Equation (6)
  options += "Numeric tol " + std::to_string(tol) + "\n";
  options += "Numeric acceptable_tol " + std::to_string(acceptable_tol) + "\n";
  // maximum amount of random pertubation; e.g.,
  // when evaluation finite diff
  options += "Numeric point_perturbation_radius " + std::to_string(0.05) + "\n";
  options += "String check_derivatives_for_naninf no\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, CosThetaIpoptInterface>(
      options, x, x_l, x_u, g_l, g_u, *smoother, solution);

  //
  // Check some of the solution values
  //

  bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  ADEBUG("found optimal solution: " << (solution.status == 1 ? "yes" : "no"));

  if (ok) {
    for (size_t i = 0; i < raw_point2d.size(); i++) {
      size_t index = i << 1;
      opt_x->push_back(solution.x[index]);
      opt_y->push_back(solution.x[index + 1]);
      //   ADEBUG("new point " << i << " (" << opt_x->back() << ", "
      //                           << opt_y->back() << ")");
    }
  }

  return ok;
}
}  // namespace planning
}  // namespace autoagric
