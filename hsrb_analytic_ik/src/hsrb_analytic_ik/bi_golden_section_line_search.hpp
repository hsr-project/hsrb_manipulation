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
/// @brief A class that performs bidirectional golden section line search
#ifndef HSRB_ANALYTIC_IK_BI_GOLDEN_SECTION_LINE_SEARCH_HPP_
#define HSRB_ANALYTIC_IK_BI_GOLDEN_SECTION_LINE_SEARCH_HPP_

#include "common.hpp"
#include "function_adapters.hpp"
#include "uni_golden_section_line_search.hpp"

namespace opt {

/**
 * This is a class that performs bidirectional golden section line search.
 */
class BiGoldenSectionLineSearch {
 public:
  /**
   * Constructor
   */
  explicit BiGoldenSectionLineSearch(int max_iteration = 100, double epsilon = 1e-4) {
    max_iteration_ = max_iteration;
    epsilon_ = epsilon;
    result_ = OptFail;
    iteration_ = 0;
    solution_ = 0;
    value_ = 0;
  }

  /**
   * Searches within the range (-∞,+∞).
   *
   * @param	f		A single-variable function
   * @param	step	Initial value of the search interval [-step, step]
   * @return
   */
  template<class Function1>
  OptResult Search(Function1& f, double step) {
    // Performs unidirectional search in the positive direction.
    {
      UniGoldenSectionLineSearch search(max_iteration_, epsilon_);
      OptResult result = search.Search(f, step);

      // If the search fails
      if (result == OptFail) {
        result_ = OptFail;
        // Continues the search in the negative direction.
      } else {
        // If the search succeeds or the maximum number of iterations is reached
        // Records the solution and compares it with the result of the negative direction search.
        result_ = result;
        iteration_ = search.iteration();
        solution_ = search.solution();
        value_ = search.value();
      }
    }

    // Performs unidirectional search in the negative direction.
    {
      // Creates a reversed function.
      ReverseAdapterFunction1<Function1> revserseFunc(f);

      UniGoldenSectionLineSearch search(max_iteration_, epsilon_);
      OptResult result = search.Search(revserseFunc, step);

      // If the search fails, returns the result of the positive direction search.
      if (result == OptFail) {
        return result_;
      } else {
        // If the search succeeds or the maximum number of iterations is reached
        // If the positive direction search failed, or if it succeeded but
        // the negative direction yielded a better result, that result is taken as the final solution.
        if ((result_ == OptFail) || (search.value() < value_)) {
          result_ = result;
          iteration_ = search.iteration();
          solution_ = -search.solution();  // Note that a negative sign is applied
          value_ = search.value();
        }
        return result_;
      }
    }
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
   * Retrieves the search result.
   */
  OptResult result() const {
    return result_;
  }

  /**
   * Retrieves the number of iterations taken for the search.
   */
  int iteration() const {
    return iteration_;
  }

  /**
   * Retrieves the solution after the search.
   */
  double solution() const {
    return solution_;
  }

  /**
   * Retrieves the objective function value after the search.
   */
  double value() const {
    return value_;
  }

 private:
  OPT_CLASS_UNCOPYABLE(BiGoldenSectionLineSearch)

  /**
   * Maximum number of iterations
   */
  int max_iteration_;

  /**
   * Convergence condition
   * (Convergence is considered achieved if the width of the uncertain interval becomes less than this value)
   */
  double epsilon_;

  /**
   * Holds the search result.
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
#endif  // HSRB_ANALYTIC_IK_BI_GOLDEN_SECTION_LINE_SEARCH_HPP_
