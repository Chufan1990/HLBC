#pragma once

#include <vector>
#include <memory>

#include "Eigen/Core"
#include "autoagric/common/pnc_point.pb.h"
#include "autoagric/common/vehicle_config.pb.h"
#include "autoagric/control/mpc_controller_conf.pb.h"
#include "control/common/dependency_injector.h"
#include "cppad/cppad.hpp"

namespace autoagric {
namespace common {
namespace math {
class MpcIpopt {
 public:
  typedef CPPAD_TESTVECTOR(double) Dvector;
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

  MpcIpopt() = default;

  MpcIpopt(MpcIpopt* other);

  MpcIpopt(const std::string options, const int horizon, const double dt,
           const double lf,
           const control::MPCControllerConf& mpc_controller_conf);

  void Update(Dvector* vars, Dvector* vars_lowerbound, Dvector* vars_upperbound,
              Dvector* constraints_lowerbound, Dvector* constraints_upperbound,
              Eigen::MatrixXd* matrix_q, Eigen::MatrixXd* matrix_r,
              std::vector<autoagric::common::TrajectoryPoint>* ref_trajectory);

  static bool Solve(std::shared_ptr<MpcIpopt>& mpc_ipopt,
                    std::vector<autoagric::common::TrajectoryPoint>& solution);

  void operator()(ADvector& fg, const ADvector& vars);

 private:
  Eigen::MatrixXd* matrix_q_;
  Eigen::MatrixXd* matrix_r_;
  const double lf_;
  const double horizon_;
  const double dt_;
  std::string options_;
  std::vector<autoagric::common::TrajectoryPoint>* ref_trajectory_;
  const size_t x_index_;
  const size_t y_index_;
  const size_t heading_index_;
  const size_t speed_index_;
  const size_t steer_index_;
  const size_t accel_index_;
  Dvector* vars_;
  Dvector* vars_lowerbound_;
  Dvector* vars_upperbound_;
  Dvector* constraints_lowerbound_;
  Dvector* constraints_upperbound_;
};

}  // namespace math
}  // namespace common
}  // namespace autoagric