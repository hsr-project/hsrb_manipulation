/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @brief 2D vector class
#ifndef HSRB_ANALYTIC_IK_VECTOR_2_HPP_
#define HSRB_ANALYTIC_IK_VECTOR_2_HPP_

#include <cmath>

namespace opt {

/**
 * This is a 2D vector class.
 */
struct Vector2 {
  double v1;
  double v2;

  /**
   * Constructor
   */
  Vector2() {
  }

  /**
   * Constructor
   */
  Vector2(double a1, double a2)
      : v1(a1), v2(a2) {
  }

  /**
   * Sets the values.
   */
  void Set(double a1, double a2) {
    v1 = a1;
    v2 = a2;
  }

  /**
   * Sets to a zero vector.
   */
  void Zero() {
    v1 = 0;
    v2 = 0;
  }

  /**
   * Normalizes to a unit vector.
   */
  void Normalize(void) {
    double a = 1 / Norm();
    v1 *= a;
    v2 *= a;
  }

  /**
   * Calculates the norm.
   */
  double Norm(void) {
    return std::sqrt(v1 * v1 + v2 * v2);
  }

  /**
   * Calculates the squared norm.
   */
  double Norm2(void) {
    return (v1 * v1 + v2 * v2);
  }

  /**
   * Calculates the norm between two points.
   */
  static double Norm(const Vector2& x, const Vector2& y) {
    double d1 = x.v1 - y.v1;
    double d2 = x.v2 - y.v2;
    return std::sqrt(d1 * d1 + d2 * d2);
  }

  /**
   * Creates the midpoint between two points.
   */
  static Vector2 Mid(const Vector2& x, const Vector2& y) {
    return Vector2((x.v1 + y.v1) * 0.5, (x.v2 + y.v2) * 0.5);
  }

  /**
   * Calculates the inner product x'*y.
   */
  static double Dot(const Vector2& x, const Vector2& y) {
    return (x.v1 * y.v1 + x.v2 * y.v2);
  }
};

inline Vector2 operator+(const Vector2& x, const Vector2& y) {
  return Vector2(x.v1 + y.v1, x.v2 + y.v2);
}

inline Vector2 operator-(const Vector2& x, const Vector2& y) {
  return Vector2(x.v1 - y.v1, x.v2 - y.v2);
}

inline Vector2 operator-(const Vector2& x) {
  return Vector2(-x.v1, -x.v2);
}

inline Vector2 operator*(double a, const Vector2& x) {
  return Vector2(a * x.v1, a * x.v2);
}

inline Vector2 operator*(const Vector2& x, double a) {
  return Vector2(x.v1 * a, x.v2 * a);
}

inline Vector2 operator/(const Vector2& x, double a) {
  double b = 1 / a;
  return Vector2(x.v1 * b, x.v2 * b);
}

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_VECTOR_2_HPP_
