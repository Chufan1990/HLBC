#include "planning/math/discretized_points_smoothing/cos_theta_ipopt_interface.h"

#include <random>

#include "glog/logging.h"

namespace autoagric {
namespace planning {

using Dvector = CPPAD_TESTVECTOR(double);
using ADvector = CPPDA_TESTVECTOR(CppAD::AD(double));

CosThetaIpoptInterface::CosThetaIpoptInterface(
    std::vector<std::pair<double, double>> point, std::vector<double> bounds) {
  CHECK_GT(points.size(), 1U);
  CHECK_GT(bounds.size(), 1U);
  bounds_ = std::move(bounds);
  ref_points_ = std::move(points);
  num_of_points_ = ref_points_.size();

  // number of variables
  n = static_cast<int>(num_of_points_ << 1);
  num_of_variables_ = n;

  // number of constraints
  m = static_cast<int>(num_of_points_ << 1);
  num_of_constraints_ = m;
}

void CosThetaIpoptInterface::set_weight_cos_included_angle(
    const double weight_cos_included_angle) {
  weight_cos_included_angle_ = weight_cos_included_angle;
}

void CosThetaIpoptInterface::set_weight_anchor_points(
    const double weight_anchor_points) {
  weight_anchor_points_ = weight_anchor_points;
}

void CosThetaIpoptInterface::set_weight_length(const double weight_length) {
  weight_length_ = weight_length;
}

void CosThetaIpoptInterface::get_optimization_results(
    std::vector<double>* ptr_x, std::vector<double>* ptr_y) const {
  *ptr_x = opt_x_;
  *ptr_y = opt_y_;
}

bool CosThetaIpoptInterface::get_bounds_info(int n, Dvector& x_l, Dvector& x_u,
                                             int m, Dvector& g_l,
                                             Dvector& g_u) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);
  CHECK_EQ(static_cast<size_t>(m), num_of_constraints_);
  // variables
  // a. for x, y
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    // x
    x_l[index] = -1e20;
    x_u[index] = 1e20;

    // y
    x_l[index + 1] = -1e20;
    x_u[index + 1] = 1e20;
  }

  // constraints
  // positional deviation constraints
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    double x_lower = 0.0;
    double x_upper = 0.0;
    double y_lower = 0.0;
    double y_upper = 0.0;

    x_lower = ref_points_[i].first - bounds_[i];
    x_upper = ref_points_[i].first + bounds_[i];
    y_lower = ref_points_[i].second - bounds_[i];
    y_upper = ref_points_[i].second + bounds_[i];

    // x
    g_l[index] = x_lower;
    g_u[index] = x_upper;

    // y
    g_l[index + 1] = y_lower;
    g_u[index + 1] = y_upper;
  }
  return true;
}

bool CosThetaIpoptInterface::get_starting_point(int n, Dvector& x) {
  CHECK_EQ(static_cast<size_t>(n), num_of_variables_);

  std::random_device rd;
  std::default_random_engine gen = std::default_random_engine(rd());
  std::normal_distribution<> dis{0, 0.05};
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    x[index] = ref_points_[i].first + dis(gen);
    x[index + 1] = ref_points_[i].second + dis(gen);
  }
  return true;
}

void CosThetaIpoptInterface::operator()(ADvector& fg, const ADvetor& x) {
  CHECK_EQ(fg.size(), num_of_variables_ + num_of_constraints_ + 1);
  CHECK_EQ(x.size(), num_of_variables_);

  CHECK(eval_obj<ADvector>(x, fg));
  CHECK(eval_constraints<ADvector>(x, fg));
}

/** Template to compute contraints */
template <class T>
bool CosThetaIpoptInterface::eval_constraints(const T& x, T& g) {
  // fill in the positional deviation constraints
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    g[index + 1] = x[index];
    g[index + 2] = x[index + 1];
  }
  return true;
}

template <class T>
bool CosThetaIpoptInterface::eval_obj(int n, const T& x, T& obj_value) {
  obj_value[0] = 0.0;
  for (size_t i = 0; i < num_of_points_; ++i) {
    size_t index = i << 1;
    obj_value[0] +=
        weight_anchor_points_ *
        ((x[index] - ref_points_[i].first) * (x[index] - ref_points_[i].first) +
         (x[index + 1] - ref_points_[i].second) *
             (x[index + 1] - ref_points_[i].second));
  }
  for (size_t i = 0; i < num_of_points_ - 2; ++i) {
    size_t findex = i << 1;
    size_t mindex = findex + 2;
    size_t lindex = mindex + 2;
    obj_value[0] -=
        weight_cos_included_angle_ *
        (((x[mindex] - x[findex]) * (x[lindex] - x[mindex])) +
         ((x[mindex + 1] - x[findex + 1]) * (x[lindex + 1] - x[mindex + 1]))) /
        (CppAD::sqrt((x[mindex] - x[findex]) * (x[mindex] - x[findex]) +
                     (x[mindex + 1] - x[findex + 1]) *
                         (x[mindex + 1] - x[findex + 1])) *
         CppAD::sqrt((x[lindex] - x[mindex]) * (x[lindex] - x[mindex]) +
                     (x[lindex + 1] - x[mindex + 1]) *
                         (x[lindex + 1] - x[mindex + 1])));
  }

  // Total length
  for (size_t i = 0; i < num_of_points_ - 1; ++i) {
    size_t findex = i << 1;
    size_t nindex = findex + 2;
    obj_value[0] +=
        weight_length_ *
        ((x[findex] - x[nindex]) * (x[findex] - x[nindex]) +
         (x[findex + 1] - x[nindex + 1]) * (x[findex + 1] - x[nindex + 1]));
  }
  return true;
}

}  // namespace planning
}  // namespace autoagric