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
/// @brief A class for unidirectional golden section line search
#ifndef HSRB_ANALYTIC_IK_UNI_GOLDEN_SECTION_LINE_SEARCH_HPP_
#define HSRB_ANALYTIC_IK_UNI_GOLDEN_SECTION_LINE_SEARCH_HPP_

#include "common.hpp"
#include "golden_section_line_search.hpp"

namespace opt {

/**
 * This is a class for unidirectional golden section line search.
 */
class UniGoldenSectionLineSearch {
 public:
  /**
   * Constructor
   */
  explicit UniGoldenSectionLineSearch(int max_iteration = 100, double epsilon = 1e-4)
      : max_iteration_(max_iteration),
        epsilon_(epsilon),
        result_(OptFail),
        iteration_(0),
        solution_(0),
        value_(0) {}

  /**
   * Performs the search within the range (0, +∞).
   *
   * @param	f		A single-variable function
   * @param	step	Initial value of the search interval [0, step]
   * @return
   */
  template<class Function1>
  OptResult Search(Function1& f, double step) {
    // Initialize the search results.
    result_ = OptFail;
    iteration_ = 0;
    solution_ = 0;
    value_ = 0;

    // Create a golden section line search.
    GoldenSectionLineSearch search(max_iteration_, epsilon_);

    // Initialize the search interval.
    const double a = 0;
    double b = step;

    // Perform the first search.
    OptResult result = search.Search(f, a, b);

    // If the search is successful, record the solution.
    iteration_ = search.iteration();
    solution_ = search.solution();
    value_ = search.value();
    if (result == OptSuccess) {
      result_ = OptSuccess;
      // Continue expanding the interval to search for a solution.
    } else if (result == OptMaxItor) {
      result_ = OptMaxItor;
      return result_;
    }

    // If non-quasi-convexity is detected in the first golden section search,
    // reduce the step size to make the interval quasi-convex.
    if (result == OptFail) {
      // Repeat iteration k.
      for (int k = 2; k <= max_iteration_; k++) {
        // Shrink the interval.
        b *= 0.5;

        // Perform the search.
        OptResult result = search.Search(f, a, b);

        if (result == OptSuccess) {
          result_ = OptSuccess;
          iteration_ = search.iteration();
          solution_ = search.solution();
          value_ = search.value();
          return result_;
        }
      }

      result_ = OptMaxItor;
      return result_;
    } else {
      // Repeat iteration k.
      for (int k = 2; k <= max_iteration_; k++) {
        // Expand the interval.
        b *= 2;

        // Perform the search.
        OptResult result = search.Search(f, a, b);

        if (result == OptSuccess) {
          double comparativeSolution = search.solution();
          double comparativeValue = search.value();

          // If the previous solution and the current solution are sufficiently close,
          // choose the one with the smaller objective function value as the solution.
          // Judging only by the objective function value without this step,
          // may lead to unnecessary iterations due to minor computational errors.
          if (std::abs(solution_ - comparativeSolution) <= epsilon_) {
            if (comparativeValue < value_) {
              iteration_ = search.iteration();
              solution_ = comparativeSolution;
              value_ = comparativeValue;
            }
            return result_;
          } else if (comparativeValue < value_) {
            // If the current solution and the previous solution are far apart,
            // and the objective function value has decreased, continue the search.
            iteration_ = search.iteration();
            solution_ = comparativeSolution;
            value_ = comparativeValue;
            continue;
          } else {
            // Return the previously successful solution.
            return result_;
          }
        } else {
          // In case of OptMaxItor or OptFail
          // Return the previously successful solution.
          return result_;
        }
      }

      // Since the search has succeeded at least once, use that as the solution.
      return result_;
    }
  }

  /**
   * Set the maximum number of iterations.
   */
  void set_max_iteration(int max_iteration) {
    max_iteration_ = max_iteration;
  }

  /**
   * Set the convergence criteria.
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
   * Retrieve the number of iterations taken for the search.
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
  OPT_CLASS_UNCOPYABLE(UniGoldenSectionLineSearch)

  /**
   * Maximum number of iterations
   */
  int max_iteration_;

  /**
   * Convergence criteria
   * (Convergence is considered achieved if the width of the uncertain interval becomes less than or equal to this value)
   */
  double epsilon_;

  /**
   * Holds the search results.
   */
  OptResult result_;

  /**
   * Holds the number of iterations taken for the search.
   */
  int iteration_;

  /**
   * Holds the solution after the search.
   */
  double solution_;

  /**
   * Holds the objective function value of the solution.
   */
  double value_;
};

}  // namespace opt
#endif  // HSRB_ANALYTIC_IK_UNI_GOLDEN_SECTION_LINE_SEARCH_HPP_
