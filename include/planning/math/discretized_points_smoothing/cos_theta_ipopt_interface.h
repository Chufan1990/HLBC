/**
 * @file
 */

#pragma once

#include <cppad/cppad.hpp>
#include <utility>
#include <vector>

namespace autoagric {
namespace planning {

class CosThetaIpoptInterface {
 public:
  typedef CPPAD_TESTVECTOR(double) Dvector;
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

  CosThetaIpoptInterface(const std::vector<std::pair<double, double>>& point,
                         const std::vector<double>& bounds);

  virtual ~CosThetaIpoptInterface() = default;

  void set_weight_cos_included_angle(const double weight_cos_included_angle);

  void set_weight_anchor_points(const double weight_anchor_points);

  void set_weight_length(const double weight_length);

  void get_optimization_results(std::vector<double>* ptr_x,
                                std::vector<double>* ptr_y) const;

  bool get_bounds_info(int n, Dvector& x_l, Dvector& x_u, int m, Dvector& g_l,
                       Dvector& g_u);

  bool get_starting_point(int n, Dvector& x);

  void operator()(ADvector& fg, const ADvector& x);

 private:
  template <class T>
  bool eval_obj(const T& x, T& obj_value);

  template <class T>
  bool eval_constraints(const T& x, T& g);

  std::vector<std::pair<double, double>> ref_points_;

  std::vector<double> bounds_;

  std::vector<double> opt_x_;

  std::vector<double> opt_y_;

  size_t num_of_variables_ = 0;

  size_t num_of_constraints_ = 0;

  size_t num_of_points_ = 0;

  double weight_cos_included_angle_ = 0.0;

  double weight_anchor_points_ = 0.0;

  double weight_length_ = 0.0;
};

}  // namespace planning
}  // namespace autoagric