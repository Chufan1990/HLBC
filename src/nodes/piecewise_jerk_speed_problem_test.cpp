#include "planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

#include <utility>
#include <vector>

#include "common/macro.h"
#include "planning/common/speed/speed_data.h"

int main(int argc, char** argv) {
  const bool gear = true;
  const double init_v = 0.0;
  const double init_a = 0.0;

  const double max_forward_v = 2.0;
  const double max_reverse_v = 1.0;
  const double max_forward_a = 1.0;
  const double max_reverse_a = 1.0;
  const double max_acc_jerk = 1.0;
  const double dt = 5.0;
  const double time_slack_ratio = 1.5;
  const double max_path_time = 10.0;

  autoagric::planning::SpeedData speed_data;

  const double path_length = 15.0;
  const double total_time = std::max(
      gear ? time_slack_ratio *
                 (max_forward_v * max_forward_v + path_length * max_forward_a) /
                 (max_forward_v * max_forward_a)
           : time_slack_ratio *
                 (max_reverse_v * max_reverse_v + path_length * max_reverse_a) /
                 (max_reverse_a * max_reverse_v),
      max_path_time);

  const size_t num_of_knots = static_cast<size_t>(total_time / dt) + 1;

  autoagric::planning::PiecewiseJerkSpeedProblem piecewise_jerk_problem(
      num_of_knots, dt, {0.0, std::abs(init_v), std::abs(init_a)});

  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, path_length});
  const double max_v = gear ? max_forward_v : max_reverse_v;
  const double max_a = gear ? max_forward_a : max_reverse_a;

  const auto upper_dx = std::max(max_v, std::abs(init_v));
  const auto upper_ddx = std::max(max_a, std::abs(init_a));

  std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                   {0.0, upper_dx});
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {-upper_ddx, upper_ddx});

  // x_bounds[0] = std::make_pair(0.0, 0.0);
  x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
  // dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
  // ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

  std::vector<double> x_ref(num_of_knots, path_length);

  const double weight_x_ref = 1000.0;
  const double weight_ddx = 10.0;
  const double weight_dddx = 10.0;

  // for (size_t i = 0; i < num_of_knots; i++) {
  //   ADEBUG("x_ref: " << x_ref[i]);
  //   ADEBUG("x_bounds: " << x_bounds[i].first << " " << x_bounds[i].second);
  //   ADEBUG("dx_bounds: " << dx_bounds[i].first << " " <<
  //   dx_bounds[i].second); ADEBUG("ddx_bounds: " << ddx_bounds[i].first << " "
  //                         << ddx_bounds[i].second);
  //
  ADEBUG("num_of_knots " << num_of_knots);
  ADEBUG("path_length " << path_length);
  ADEBUG("upper_dx " << upper_dx);
  ADEBUG("upper_ddx " << upper_ddx);
  ADEBUG("total_time " << total_time);
  ADEBUG("max_acc_jerk " << max_acc_jerk);

  piecewise_jerk_problem.set_x_ref(weight_x_ref, std::move(x_ref));
  piecewise_jerk_problem.set_weight_dx(0.0);
  piecewise_jerk_problem.set_weight_ddx(weight_ddx);
  piecewise_jerk_problem.set_weight_dddx(weight_dddx);
  piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);
  // piecewise_jerk_problem.set_scale_factor({1.0, 2.0, 3.0});
  // std::array<double, 3> scale_factor_ = {{10.0, 100.0, 10000.0}};

  if (!piecewise_jerk_problem.Optimize(100000)) {
    AERROR("Piecewise jerk speed optimizer failed");
    return false;
  }

  const auto& s = piecewise_jerk_problem.opt_x();
  const auto& ds = piecewise_jerk_problem.opt_dx();
  const auto& dds = piecewise_jerk_problem.opt_ddx();

  return 0;
}