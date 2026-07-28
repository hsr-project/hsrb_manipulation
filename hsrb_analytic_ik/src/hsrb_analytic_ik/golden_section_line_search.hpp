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
/// @brief Class for performing the golden section line search method
#ifndef HSRB_ANALYTIC_IK_GOLDEN_SECTION_LINE_SEARCH_HPP_
#define HSRB_ANALYTIC_IK_GOLDEN_SECTION_LINE_SEARCH_HPP_

#include "common.hpp"

namespace opt {

/**
 * This is a class for performing the golden section line search method.
 */
class GoldenSectionLineSearch {
 public:
  /**
   * Constructor
   */
  explicit GoldenSectionLineSearch(int max_iteration = 100, double epsilon = 1e-4)
      : max_iteration_(max_iteration),
        epsilon_(epsilon),
        result_(OptFail),
        iteration_(0),
        solution_(0),
        value_(0) {}

  /**
   * Performs the search.
   *
   * If the single-variable function f is strictly convex in the search interval [a, b],
   * The search will converge to the minimum value.
   *
   * If strict convexity is detected as violated during the search, the search fails and returns OptFail.
   * However, detection is not guaranteed, and if not detected, a local solution is obtained.
   *
   * @param	f		Single-variable function
   * @param	a		Lower bound of the search interval [a, b]
   * @param	b		Upper bound of the search interval [a, b]
   * @param	maxItor	Maximum number of iterations
   * @param	epsilon	Convergence criterion. If the width of the uncertain interval becomes less than this value, it is considered converged.
   * @return
   */
  template<class Function1>
  OptResult Search(Function1& f, double a, double b) {
    // Initialize the search results.
    result_ = OptFail;
    iteration_ = 0;
    solution_ = 0;
    value_ = 0;

    // Initialize the uncertain interval.
    double a_k = a;
    double b_k = b;

    // Calculate the function values f_a_k, f_b_k at the endpoints a_k, b_k of the uncertain interval.
    double f_a_k = f.Value(a_k);
    double f_b_k = f.Value(b_k);

    // Set the golden ratio.
    const double _alpha = 0.61803398874989484820458683436563811772;

    // Select the evaluation points s_k, t_k within the uncertain interval [a_k, b_k].
    double s_k = a_k + (1 - _alpha) * (b_k - a_k);
    double t_k = a_k + _alpha * (b_k - a_k);

    // Calculate the function values f_s_k, f_t_k at the evaluation points s_k, t_k.
    double f_s_k = f.Value(s_k);
    double f_t_k = f.Value(t_k);

    // Repeat iteration k.
    for (int k = 1; k <= max_iteration_; k++) {
      // Flag indicating whether to select [a_k, t_k] as the next uncertain interval (true),
      // or [s_k, b_k] (false).
      bool left = (f_a_k <= f_s_k && f_a_k <= f_t_k && f_a_k <= f_b_k)
               || (f_s_k <= f_a_k && f_s_k <= f_t_k && f_s_k <= f_b_k);

      // Determine the next uncertain interval.
      if (left) {
        // a_k remains unchanged.
        b_k = t_k;
        f_b_k = f_t_k;
        t_k = s_k;
        f_t_k = f_s_k;
        s_k = a_k + (1 - _alpha) * (b_k - a_k);
        f_s_k = f.Value(s_k);
      } else {
        // b_k remains unchanged.
        a_k = s_k;
        f_a_k = f_s_k;
        s_k = t_k;
        f_s_k = f_t_k;
        t_k = a_k + _alpha * (b_k - a_k);
        f_t_k = f.Value(t_k);
      }

      // Evaluate the convergence criterion.
      if (b_k - a_k <= epsilon_) {
        result_ = OptSuccess;
        iteration_ = k;
        if (f_a_k <= f_b_k) {
          solution_ = a_k;
          value_ = f_a_k;
        } else {
          solution_ = b_k;
          value_ = f_b_k;
        }
        return result_;
      }
    }

    // Maximum number of iterations reached, returning OptMaxItor.
    result_ = OptMaxItor;
    iteration_ = max_iteration_;
    if (f_a_k <= f_b_k) {
      solution_ = a_k;
      value_ = f_a_k;
    } else {
      solution_ = b_k;
      value_ = f_b_k;
    }
    return result_;
  }

  /**
   * Set the maximum number of iterations.
   */
  void set_max_iteration(int max_iteration) {
    max_iteration_ = max_iteration;
  }

  /**
   * Set the convergence criterion.
   */
  void set_epsilon(double epsilon) {
    epsilon_ = epsilon;
  }

  /**
   * Retrieve the search results.
   */
  OptResult result() const {
    return result_;
  }

  /**
   * Retrieve the number of iterations taken during the search.
   */
  int iteration() const {
    return iteration_;
  }

  /**
   * Retrieve the solution after the search.
   */
  double solution() const {
    return solution_;
  }

  /**
   * Retrieve the objective function value after the search.
   */
  double value() const {
    return value_;
  }

 private:
  OPT_CLASS_UNCOPYABLE(GoldenSectionLineSearch)

  /**
   * Maximum number of iterations
   */
  int max_iteration_;

  /**
   * Convergence criterion
   */
  double epsilon_;

  /**
   * Holds the search results.
   */
  OptResult result_;

  /**
   * Holds the number of iterations taken during the search.
   */
  int iteration_;

  /**
   * Holds the solution after the search.
   */
  double solution_;

  /**
   * Holds the objective function value after the search.
   */
  double value_;
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_GOLDEN_SECTION_LINE_SEARCH_HPP_
