/**
 * @brief Defines the Vec2d class
 */

#pragma once

#include <string>

/**
 * @namespace common::math
 * @brief common::math
 */
namespace autoagric {
namespace common {
namespace math {

constexpr double kMathEpsilon = 1e-10;

/**
 * @class Vec2d
 * @brief Implements a class of 2-dimensinal vectors
 */

class Vec2d {
 public:
  /**
   * @brief constructor. keyword constexpr declareds calculation completes in
   * compiling time. specifier noexcept declares this function to not throw any
   * exceptions.
   */
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}

  /**
   * @brief delegating constructor returning the zero vector
   */
  constexpr Vec2d() noexcept : Vec2d(0, 0) {}

  /**
   * @brief create a unit-vector with a given angle to the positive x semi-axis
   * @param angle the desired direction
   * @return 2d-vector towards the given direction
   */
  static Vec2d CreateUnitVec2d(const double angle);

  double x() const { return x_; }

  double y() const { return y_; }

  void set_x(const double x) { x_ = x; }

  void set_y(const double y) { y_ = y; }
  /**
   * @brief calculate the length of the vector
   * @return length of the vector
   */
  double Length() const;

  /**
   * @brief calculate the squared length of the vector
   * @return square of the length of the vector
   */
  double LengthSquare() const;

  /**
   * @brief calculate the angle of the vector
   * @return angle of the vector
   */
  double Angle() const;

  /**
   * @brief normlize the vector to the co-linear unit Vec2d
   */
  void Normalize();

  /**
   * @brief the distance to a given Vec2d
   */
  double DistanceTo(const Vec2d &other) const;

  /**
   * @brief square of the distance to a given Vec2d
   */
  double DistanceSquareTo(const Vec2d &other) const;

  /**
   * @brief the "cross" product between these two Vec2d (non-standard).
   */
  double CrossProd(const Vec2d &other) const;

  /**
   * @brief the inner product between these two Vec2d.
   */
  double InnerProd(const Vec2d &other) const;

  /**
   * @brief rotate the vector by angle.
   */
  Vec2d rotate(const double angle) const;

  /**
   * @brief rotate the vector itself by angle.
   */
  void SelfRotate(const double angle);

  /**
   * @brief Sums two Vec2d
   */
  Vec2d operator+(const Vec2d &other) const;

  /**
   * @brief Subtracts two Vec2d
   */
  Vec2d operator-(const Vec2d &other) const;

  /**
   * @brief Multiplies Vec2d by a scalar
   */
  Vec2d operator*(const double ratio) const;

  /**
   * @brief Divides Vec2d by a scalar
   */
  Vec2d operator/(const double ratio) const;

  /**
   * @brief Sums another Vec2d to the current one
   */
  Vec2d &operator+=(const Vec2d &other);

  /**
   * @brief Subtracts another Vec2d to the current one
   */
  Vec2d &operator-=(const Vec2d &other);

  /**
   * @brief Multiplies this Vec2d by a scalar
   */
  Vec2d &operator*=(const double ratio);

  /**
   * @brief Divides this Vec2d by a scalar
   */
  Vec2d &operator/=(const double ratio);

  /**
   * @brief Compares two Vec2d
   */
  bool operator==(const Vec2d &other) const;

  /**
   * @brief Returns a human-readable string representing this object
   */
  std::string DebugString() const;

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

}  // namespace math
}  // namespace common
}  // namespace autoagric