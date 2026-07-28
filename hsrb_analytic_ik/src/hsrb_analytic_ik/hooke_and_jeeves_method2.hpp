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
/// @brief Class for optimization using the Hooke-and-Jeeves method
#ifndef HSRB_ANALYTIC_IK_HOOKE_AND_JEEVES_METHOD_2_HPP_
#define HSRB_ANALYTIC_IK_HOOKE_AND_JEEVES_METHOD_2_HPP_

#include "common.hpp"
#include "function_adapters.hpp"

namespace opt {

/**
 * This is a class for optimization using the Hooke-and-Jeeves method.
 * It is an optimization method that does not use gradients.
 */
class HookeAndJeevesMethod2 {
 public:
  /**
   * Constructor
   */
  explicit HookeAndJeevesMethod2(int max_iteration = 100,
                                 double epsilon = 1e-4)
      : max_iteration_(max_iteration), epsilon_(epsilon) {
  }

  /**
   * Performs exploration.
   *
   * @param	func	Objective function
   * @param	x0		Initial value
   * @param	step	Step width (used for line search)
   * @param	maxItor	Maximum iterations
   * @param	epsilon	Convergence condition (considered converged when the distance moved by iteration is less than this distance)
   */
  template<class Function2, class BiLineSearch>
  OptResult Search(Function2& func, BiLineSearch& lineSearch, const Vector2& x0,
                   double step) {
    // Initializes the exploration results.
    result_ = OptFail;
    iteration_ = 0;
    solution_.Zero();

    // Initializes the point sequence.
    Vector2 x = x0;

    // Initializes the previous position z_prev.
    Vector2 z_prev = x0;

    // Repeats iteration k.
    for (int k = 1; k <= max_iteration_; k++) {
      // Performs a line search from point x in the (1,0) direction to determine point y.
      Vector2 y;
      {
        DirectionAdapterFunction2<Function2> func_x1(func, x, Vector2(1, 0));
        OptResult result_x1 = lineSearch.Search(func_x1, step);
        if (result_x1 == OptFail) {
          result_ = OptFail;
          return result_;
        }
        double solution_x1 = lineSearch.solution();
        y.v1 = x.v1 + solution_x1;
        y.v2 = x.v2;
      }

      // Performs a line search from point y in the (0,1) direction to determine point z.
      Vector2 z;
      {
        DirectionAdapterFunction2<Function2> func_x2(func, y, Vector2(0, 1));
        OptResult result_x2 = lineSearch.Search(func_x2, step);
        if (result_x2 == OptFail) {
          result_ = OptFail;
          return result_;
        }
        double solution_x2 = lineSearch.solution();
        z.v1 = y.v1;
        z.v2 = y.v2 + solution_x2;
      }

      // Performs pattern search using the Hooke-and-Jeeves method.
      // Instead of searching in the direction d=z-x based on x before x1, x2 direction search,
      // Searches in the direction d=z-z_prev based on the previous position z_prev.
      Vector2 d = z - z_prev;

      // Ends iteration if point z and point z_prev are sufficiently close.
      if (d.Norm() <= epsilon_) {
        result_ = OptSuccess;
        iteration_ = k;
        solution_ = (func.Value(z) <= func.Value(z_prev)) ? z : z_prev;
        return result_;
      }

      // Performs a line search from point z in direction d to determine point x_next.
      Vector2 x_next;
      {
        d.Normalize();
        DirectionAdapterFunction2<Function2> func_d(func, z, d);
        OptResult result_d = lineSearch.Search(func_d, step);
        if (result_d == OptFail) {
          result_ = OptFail;
          return result_;
        }
        double solution_d = lineSearch.solution();
        x_next = z + solution_d * d;
      }

      // Calculates the distance between point x and point x_next.
      double dx = Vector2::Norm(x, x_next);

      // Ends iteration if point x and point x_next are sufficiently close.
      if (dx <= epsilon_) {
        result_ = OptSuccess;
        iteration_ = k;
        solution_ = (func.Value(x_next) <= func.Value(x)) ? x_next : x;
        return result_;
      }

      // Records the previous point z.
      z_prev = z;

      // Updates the focus point x.
      x = x_next;
    }

    // Returns OptMaxItor as the maximum number of iterations has been reached.
    result_ = OptMaxItor;
    iteration_ = max_iteration_;
    solution_ = x;
    return result_;
  }

  /**
   * Sets the maximum number of iterations.
   */
  void set_max_iteration(int max_iteration) {
    max_iteration_ = max_iteration;
  }

  /**
   * Sets the convergence condition.
   */
  void set_epsilon(double epsilon) {
    epsilon_ = epsilon;
  }

  /**
   * Retrieves the exploration results.
   */
  OptResult result() const {
    return result_;
  }

  /**
   * Retrieves the number of iterations taken for exploration.
   */
  int iteration() const {
    return iteration_;
  }

  /**
   * Retrieves the solution after exploration.
   */
  Vector2 solution() const {
    return solution_;
  }

 private:
  OPT_CLASS_UNCOPYABLE(HookeAndJeevesMethod2)

  /**
   * Maximum number of iterations
   */
  int max_iteration_;

  /**
   * Convergence condition
   * (Considered converged when the distance moved by iteration is less than this distance)
   */
  double epsilon_;

  /**
   * Holds the exploration results.
   */
  OptResult result_;

  /**
   * Holds the number of iterations taken for exploration.
   */
  int iteration_;

  /**
   * Holds the solution after exploration.
   */
  Vector2 solution_;
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_HOOKE_AND_JEEVES_METHOD_2_HPP_
