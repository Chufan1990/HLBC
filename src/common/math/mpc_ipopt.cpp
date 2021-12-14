#include "common/math/mpc_ipopt.h"

#include "common/macro.h"

namespace autoagric {
namespace common {
namespace math {

using CppAD::AD;
using CppAD::cos;
using CppAD::pow;
using CppAD::sin;
using Matrix = Eigen::MatrixXd;

MpcIpopt::MpcIpopt(MpcIpopt* other)
    : options_(other->options_),
      horizon_(other->horizon_),
      dt_(other->dt_),
      lf_(other->lf_),
      x_index_(other->x_index_),
      y_index_(other->y_index_),
      heading_index_(other->heading_index_),
      speed_index_(other->speed_index_),
      steer_index_(other->steer_index_),
      accel_index_(other->accel_index_) {
  vars_lowerbound_ = other->vars_lowerbound_;
  vars_upperbound_ = other->vars_upperbound_;
  constraints_lowerbound_ = other->constraints_lowerbound_;
  constraints_upperbound_ = other->constraints_upperbound_;
  matrix_q_ = other->matrix_q_;
  matrix_r_ = other->matrix_r_;
  vars_ = other->vars_;
  ref_trajectory_ = other->ref_trajectory_;
}

MpcIpopt::MpcIpopt(const std::string options, const int horizon,
                   const double dt, const double lf,
                   const control::MPCControllerConf& mpc_controller_conf)
    : options_(options),
      horizon_(horizon),
      dt_(dt),
      lf_(lf),
      x_index_(0),
      y_index_(x_index_ + horizon),
      heading_index_(y_index_ + horizon),
      speed_index_(heading_index_ + horizon),
      steer_index_(speed_index_ + horizon),
      accel_index_(steer_index_ + horizon - 1) {
  ADEBUG("x_index_" << x_index_ << "\ny_index_" << y_index_ << "\nheading_index_"
                    << heading_index_ << "\nspeed_index_" << speed_index_
                    << "\nsteer_index_" << steer_index_ << "\naccel_index_"
                    << accel_index_);
}

void MpcIpopt::Update(
    Dvector* vars, Dvector* vars_lowerbound, Dvector* vars_upperbound,
    Dvector* constraints_lowerbound, Dvector* constraints_upperbound,
    Matrix* matrix_q, Matrix* matrix_r,
    std::vector<autoagric::common::TrajectoryPoint>* ref_trajectory) {
  vars_lowerbound_ = vars_lowerbound;
  vars_upperbound_ = vars_upperbound;
  constraints_lowerbound_ = constraints_lowerbound;
  constraints_upperbound_ = constraints_upperbound;
  matrix_q_ = matrix_q;
  matrix_r_ = matrix_r;
  vars_ = vars;
  ref_trajectory_ = ref_trajectory;
}

bool MpcIpopt::Solve(
    std::shared_ptr<MpcIpopt>& mpc_ipopt,
    std::vector<autoagric::common::TrajectoryPoint>& solution) {
  CppAD::ipopt::solve_result<Dvector> ret;

  CppAD::ipopt::solve<Dvector, MpcIpopt>(
      mpc_ipopt->options_, *(mpc_ipopt->vars_), *(mpc_ipopt->vars_lowerbound_),
      *(mpc_ipopt->vars_upperbound_), *(mpc_ipopt->constraints_lowerbound_),
      *(mpc_ipopt->constraints_upperbound_), *mpc_ipopt, ret);

  bool ok = true;
  ok &= ret.status == CppAD::ipopt::solve_result<Dvector>::success;

  if (!ok) {
    AERROR("mpc solver failed: " << ret.status);
    return ok;
  }

  if (solution.size() != mpc_ipopt->horizon_)
    solution.resize(mpc_ipopt->horizon_);

  auto& p = solution[0];
  p.mutable_path_point();

  p.mutable_path_point()->set_x(ret.x[mpc_ipopt->x_index_]);
  p.mutable_path_point()->set_y(ret.x[mpc_ipopt->y_index_]);
  p.mutable_path_point()->set_theta(ret.x[mpc_ipopt->heading_index_]);
  p.set_v(ret.x[mpc_ipopt->speed_index_]);

  for (int i = 1; i < mpc_ipopt->horizon_; i++) {
    auto& p = solution[i];
    p.mutable_path_point()->set_x(ret.x[mpc_ipopt->x_index_ + i]);
    p.mutable_path_point()->set_y(ret.x[mpc_ipopt->y_index_ + i]);
    p.mutable_path_point()->set_theta(ret.x[mpc_ipopt->heading_index_ + i]);
    p.set_v(ret.x[mpc_ipopt->speed_index_ + i]);
    p.set_steer(ret.x[mpc_ipopt->steer_index_ + i - 1]);
    p.set_a(ret.x[mpc_ipopt->accel_index_ + i - 1]);
  }

  return ok;
}

void MpcIpopt::operator()(ADvector& fg, const ADvector& vars) {
  fg[0] = 0;

  for (int i = 0; i < horizon_; i++) {
    fg[0] +=
        pow(vars[x_index_ + i] - (*ref_trajectory_)[i].path_point().x(), 2) *
        (*matrix_q_)(0, 0);
    fg[0] +=
        pow(vars[y_index_ + i] - (*ref_trajectory_)[i].path_point().y(), 2) *
        (*matrix_q_)(1, 1);
    fg[0] += pow(vars[heading_index_ + i] -
                     (*ref_trajectory_)[i].path_point().theta(),
                 2) *
             (*matrix_q_)(2, 2);
    fg[0] += pow(vars[speed_index_ + i] - (*ref_trajectory_)[i].v(), 2) *
             (*matrix_q_)(3, 3);
  }

  for (size_t i = 0; i < horizon_ - 2; i++) {
    fg[0] += pow(vars[steer_index_ + i], 2) * (*matrix_r_)(0, 0);
    fg[0] += pow(vars[accel_index_ + i], 2) * (*matrix_r_)(1, 1);
  }

  fg[1 + x_index_] = vars[x_index_];
  fg[1 + y_index_] = vars[y_index_];
  fg[1 + speed_index_] = vars[speed_index_];
  fg[1 + heading_index_] = vars[heading_index_];

  for (size_t i = 0; i < horizon_ - 1; i++) {
    AD<double> x1 = vars[x_index_ + i + 1];
    AD<double> y1 = vars[y_index_ + i + 1];
    AD<double> heading1 = vars[heading_index_ + i + 1];
    AD<double> v1 = vars[speed_index_ + i + 1];

    AD<double> x0 = vars[x_index_ + i];
    AD<double> y0 = vars[y_index_ + i];
    AD<double> heading0 = vars[heading_index_ + i];
    AD<double> v0 = vars[speed_index_ + i];

    AD<double> steer0 = vars[steer_index_ + i];
    AD<double> a0 = vars[accel_index_ + i];

    fg[2 + x_index_ + i] = x1 - (x0 + v0 * cos(heading0) * dt_);
    fg[2 + y_index_ + i] = y1 - (y0 + v0 * sin(heading0) * dt_);
    fg[2 + speed_index_ + i] = v1 - (v0 + a0 * dt_);
    fg[2 + heading_index_ + i] =
        heading1 - (heading0 + v0 * steer0 / lf_ * dt_);
  }
}

}  // namespace math
}  // namespace common
}  // namespace autoagric