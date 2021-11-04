/**
 * @file curve_math.h
 */
#pragma once

/**
 * @namespace autoagric::planning
 * @brief autoagric::planning
 */
namespace autoagric {
namespace planning {
class CurveMath {
 public:
  CurveMath() = delete;

  /**
   * @brief compute the curvature (kappa) given curve X = (x(t), y(y)) where t
   * is an arbitrary parameter.
   * @param dx dx /dt;
   * @param d2x d(dx) /dt
   * @param dy dy / dt;
   * @param d2y d(dy)/ dt;
   * @return the curvature
   */
  static double ComputeCurvature(const double dx, const double d2x,
                                 const double dy, const double d2y);

  /**
   * @brief compute the curvature change rate w.r.t curve length (dkappa) given
   * curve X = (x(t), y(t)) where t is an arbitrary paramter.
   * @param dx dx /dt;
   * @param d2x d(dx) /dt
   * @param d3x d(d2x) /dt;
   * @param dy dy / dt;
   * @param d2y d(dy)/ dt;
   * @param d3y d(d2y)/ dt;
   * @return the curvature changing rate
   */
  static double ComputeCurvatureDerivative(const double dx, const double d2x,
                                           const double d3x, const double dy,
                                           const double d2y, const double d3y);
};
}  // namespace planning
}  // namespace autoagric