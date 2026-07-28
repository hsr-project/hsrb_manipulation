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
/// @brief Test class for bidirectional golden section line search method
#include <gtest/gtest.h>

#include "bi_golden_section_line_search.hpp"
#include "functions_for_testing.hpp"

namespace opt {
////////////////////////////////////////////////////////////////////////////////
//// Test with quadratic function A

/**
 * Test if the same result is obtained for the unidirectional case with the quadratic function QuadraticFunction1A.
 */
TEST(BiGoldenSectionLineSearch_Test, QuadraticFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // Search with an initial step length of 0.1. The solution is x=1.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 0.5. The solution is x=1.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.5;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 1.0. The solution is x=1.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 1.0;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 2.0. The solution is x=1.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 2.0;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 100.0. The solution is x=1.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 100.0;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 0.01. The solution is x=1.
  {
    QuadraticFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.01;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Test with a quadratic function that has a minimum at x=-1.
  // Search with an initial step length of 0.1.
  {
    ShiftAdapterFunction1<QuadraticFunction1A> func(QuadraticFunction1A(), -2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = -1;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Test with quadratic function B

/**
 * Test if the same result is obtained for the unidirectional case with the quadratic function QuadraticFunction1B.
 */
TEST(BiGoldenSectionLineSearch_Test, QuadraticFunction1B_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // Search with an initial step length of 0.1. The solution is x=-1.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = -1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 0.5. The solution is x=-1.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.5;
    double expected = -1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 1.0. The solution is x=-1.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 1.0;
    double expected = -1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 2.0. The solution is x=-1.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 2.0;
    double expected = -1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 100.0. The solution is x=-1.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 100.0;
    double expected = -1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 0.01. The solution is x=-1.
  {
    QuadraticFunction1B func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.01;
    double expected = -1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Test with a quadratic function that has a minimum at x=2.
  // Search with an initial step length of 0.1.
  {
    ShiftAdapterFunction1<QuadraticFunction1B> func(QuadraticFunction1B(), 3.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = 2.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Test with a non-differentiable function

TEST(BiGoldenSectionLineSearch_Test, NonDiffenrentialFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // Search with an initial step length of 0.1. The solution is x=1.
  {
    // Use a function that shifts NonDiffenrentialFunction1A by +2.
    // The minimum value is at x=1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(NonDiffenrentialFunction1A(), +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.1;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 0.5. The solution is x=1.
  {
    // Use a function that shifts NonDiffenrentialFunction1A by +2.
    // The minimum value is at x=1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.5;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 1.0. The solution is x=1.
  {
    // Use a function that shifts NonDiffenrentialFunction1A by +2.
    // The minimum value is at x=1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 1.0;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 2.0. The solution is x=1.
  {
    // Use a function that shifts NonDiffenrentialFunction1A by +2.
    // The minimum value is at x=1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 2.0;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 100.0. The solution is x=1.
  {
    // Use a function that shifts NonDiffenrentialFunction1A by +2.
    // The minimum value is at x=1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 100.0;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }

  // Search with an initial step length of 0.01. The solution is x=1.
  {
    // Use a function that shifts NonDiffenrentialFunction1A by +2.
    // The minimum value is at x=1.
    ShiftAdapterFunction1<NonDiffenrentialFunction1A> func(
                                                           NonDiffenrentialFunction1A(),
                                                           +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);
    double step = 0.01;
    double expected = 1.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_NEAR(solution, expected, epsilon * 10);
  }
}

////////////////////////////////////////////////////////////////////////////////
//// Test with an inverted trapezoidal function

TEST(BiGoldenSectionLineSearch_Test, InvertedTrapeziumFunction1A_Test) {
  int maxItor = 100;
  double epsilon = 1e-8;

  // Search with an initial step length of 0.1. The solution is x∈[-1,2].
  {
    InvertedTrapeziumFunction1A func;
    BiGoldenSectionLineSearch search(maxItor, epsilon);

    double step = 0.1;
    double expected1 = -1.0;
    double expected2 = 2.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(0 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }

  // Search with an initial step length of 0.1. The solution is x∈[1,4].
  {
    // Shift the inverted trapezoidal function by +2.
    // The minimum value is at x∈[1,4].
    ShiftAdapterFunction1<InvertedTrapeziumFunction1A> func(InvertedTrapeziumFunction1A(), +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);

    double step = 0.1;
    double expected1 = 1.0;
    double expected2 = 4.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }

  // Search with an initial step length of 100. The solution is x∈[1,4].
  {
    // Shift the inverted trapezoidal function by +2.
    // The minimum value is at x∈[1,4].
    ShiftAdapterFunction1<InvertedTrapeziumFunction1A> func(
                                                            InvertedTrapeziumFunction1A(),
                                                            +2.0);
    BiGoldenSectionLineSearch search(maxItor, epsilon);

    double step = 100.0;
    double expected1 = 1.0;
    double expected2 = 4.0;

    // Perform the search.
    OptResult result = search.Search(func, step);

    // Verify the search results.
    EXPECT_EQ(OptSuccess, result);
    EXPECT_EQ(OptSuccess, search.result());

    // Verify the number of iterations.
    int iteration = search.iteration();
    EXPECT_TRUE(1 <= iteration && iteration <= maxItor);

    // Verify the solution.
    double solution = search.solution();
    EXPECT_TRUE(
                expected1 - epsilon <= solution
                && solution <= expected2 + epsilon);
  }
}

}  // namespace opt

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
