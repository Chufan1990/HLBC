/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "common/math/linear_quadratic_regulator.h"

#include <Eigen/Dense>
#include <limits>

#include "common/macro.h"

namespace autoagric {
namespace common {
namespace math {

using Matrix = Eigen::MatrixXd;

// solver with cross term
void SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                     const Matrix &R, const Matrix &M, const double tolerance,
                     const uint max_num_iteration, Matrix *ptr_K) {
  if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols() ||
      M.rows() != Q.rows() || M.cols() != R.cols()) {
    AERROR("common/math/linear_quadratic_regulator.cpp, SolveLQRProblem",
           "LQR solver: one or more matrices have incompatible dimensions.");
    return;
  }

  // ADEBUG("", "\nA: \n" << A);
  // ADEBUG("", "\nB: \n" << B);
  // ADEBUG("", "\nQ: \n" << Q);
  // ADEBUG("", "\nR: \n" << R);
  // ADEBUG("", "\nM: \n" << M);

  Matrix AT = A.transpose();
  Matrix BT = B.transpose();
  Matrix MT = M.transpose();

  // Solves a discrete-time Algebraic Riccati equation (DARE)
  // Calculate Matrix Difference Riccati Equation, initialize P and Q
  Matrix P = Q;
  uint num_iteration = 0;
  double diff = std::numeric_limits<double>::max();
  while (num_iteration++ < max_num_iteration && diff > tolerance) {
    Matrix P_next =
        AT * P * A -
        (AT * P * B + M) * (R + BT * P * B).inverse() * (BT * P * A + MT) + Q;
    // check the difference between P and P_next
    diff = (P_next - P).norm() / P_next.norm();
    P = P_next;
  }

  // ADEBUG("", "\nP: \n" << P);
  // ADEBUG("", "\nnum_iteration: \n" << num_iteration);

  if (num_iteration >= max_num_iteration) {
    ADEBUG("common/math/linear_quadratic_regulator.cpp, SolveLQRProblem",
           "LQR solver cannot converge to a solution, "
           "last consecutive result diff is: "
               << diff);
  } else {
    ADEBUG("common/math/linear_quadratic_regulator.cpp, SolveLQRProblem",
           "LQR solver converged at iteration: "
               << num_iteration << ", max consecutive result diff.: " << diff);
  }
  *ptr_K = (R + BT * P * B).inverse() * (BT * P * A + MT);
}

void SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                     const Matrix &R, const double tolerance,
                     const uint max_num_iteration, Matrix *ptr_K) {
  // create M as zero matrix of the right size:
  // M.rows() == Q.rows() && M.cols() == R.cols()
  Matrix M = Matrix::Zero(Q.rows(), R.cols());
  SolveLQRProblem(A, B, Q, R, M, tolerance, max_num_iteration, ptr_K);
}

}  // namespace math
}  // namespace common
}  // namespace autoagric
