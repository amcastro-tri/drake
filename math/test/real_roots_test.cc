#include "drake/math/real_roots.h"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace drake {
namespace math {
namespace {

using std::isfinite;
using std::isnan;

::testing::AssertionResult AreBothNaN(const std::array<double, 2>& roots) {
  if (isnan(roots[0]) && isnan(roots[1])) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "Expected both NaN, got [" << roots[0] << ", " << roots[1] << "]";
}

::testing::AssertionResult AreBothNaN(const std::array<double, 3>& roots) {
  if (isnan(roots[0]) && isnan(roots[1]) && isnan(roots[2])) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure()
         << "Expected all NaN, got [" << roots[0] << ", " << roots[1] << ", "
         << roots[2] << "]";
}

constexpr double kTol = std::numeric_limits<double>::epsilon();

GTEST_TEST(QuadraticRealRootsTest, DegenerateConstantNonZero) {
  // a = 0, b = 0, c != 0 -> no roots -> both NaN.
  const std::array<double, 2> roots = quadratic_real_roots(0.0, 0.0, 1.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(QuadraticRealRootsTest, DegenerateConstantZero) {
  // a = 0, b = 0, c = 0 -> identically zero polynomial -> first is 0, second is
  // NaN.
  const std::array<double, 2> roots = quadratic_real_roots(0.0, 0.0, 0.0);
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_TRUE(isnan(roots[1]));
}

GTEST_TEST(QuadraticRealRootsTest, LinearCase) {
  // a = 0, b != 0 -> linear root at -c / b, second is NaN.
  const std::array<double, 2> roots = quadratic_real_roots(0.0, 2.0, -6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_EQ(roots[0], 6.0 / 2.0);
  EXPECT_TRUE(isnan(roots[1]));
}

GTEST_TEST(QuadraticRealRootsTest, DoubleRootAtZero) {
  // b = 0, c = 0 -> double root at 0.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 0.0, 0.0);
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_EQ(roots[1], 0.0);
}

GTEST_TEST(QuadraticRealRootsTest, NoRealRootsFromBSymmetric) {
  // b = 0, a*c > 0 -> no real roots.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 0.0, 1.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(QuadraticRealRootsTest, SymmetricNonZeroRootsFromBSymmetric) {
  // b = 0, a*c < 0 -> roots are ±sqrt(-c/a), ordered ascending.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 0.0, -4.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_NEAR(roots[0], -2.0, kTol);
  EXPECT_NEAR(roots[1], 2.0, kTol);
}

GTEST_TEST(QuadraticRealRootsTest, CZeroNegativeFirst) {
  // c = 0 and -b/a < 0 -> {x0, 0}.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 2.0, 0.0);
  // Polynomial x^2 + 2x = x(x+2) -> roots -2 and 0.
  EXPECT_EQ(roots[0], -2.0);
  EXPECT_EQ(roots[1], 0.0);
}

GTEST_TEST(QuadraticRealRootsTest, CZeroPositiveFirst) {
  // c = 0 and -b/a >= 0 -> {0, x0}.
  const std::array<double, 2> roots = quadratic_real_roots(-1.0, 2.0, 0.0);
  // Polynomial -x^2 + 2x = -x(x-2) -> roots 0 and 2.
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_EQ(roots[1], 2.0);
}

GTEST_TEST(QuadraticRealRootsTest, GeneralNoRealRoots) {
  // Discriminant < 0.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 2.0, 5.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(QuadraticRealRootsTest, GeneralTwoRealRootsAscendingOrder) {
  //  x^2 - 5x + 6 = 0 -> roots 2 and 3.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, -5.0, 6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_NEAR(roots[0], 2.0, kTol);
  EXPECT_NEAR(roots[1], 3.0, kTol);
}

GTEST_TEST(QuadraticRealRootsTest, GeneralTwoRealRootsAlreadyInOrder) {
  // b > 0 branch: x^2 + x - 6 = 0 -> roots -3 and 2.
  const std::array<double, 2> roots = quadratic_real_roots(1.0, 1.0, -6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_NEAR(roots[0], -3.0, kTol);
  EXPECT_NEAR(roots[1], 2.0, kTol);
}

GTEST_TEST(QuadraticRealRootsTest, NonFiniteCoefficientsThrows) {
  const double inf = std::numeric_limits<double>::infinity();
  const double NaN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(quadratic_real_roots(inf, 1.0, 1.0),
               std::runtime_error);  // non-finite a
  EXPECT_THROW(quadratic_real_roots(1.0, inf, 1.0),
               std::runtime_error);  // non-finite b
  EXPECT_THROW(quadratic_real_roots(1.0, 1.0, inf),
               std::runtime_error);  // non-finite c
  EXPECT_THROW(quadratic_real_roots(NaN, 1.0, 1.0),
               std::runtime_error);  // non-finite a
  EXPECT_THROW(quadratic_real_roots(1.0, NaN, 1.0),
               std::runtime_error);  // non-finite b
  EXPECT_THROW(quadratic_real_roots(1.0, 1.0, NaN),
               std::runtime_error);  // non-finite c
}

GTEST_TEST(CubicRealRootsTest, AllNaNForNonZeroConstantOnly) {
  // a = b = c = 0, d != 0 -> constant != 0 -> no roots, all NaN.
  const std::array<double, 3> roots = cubic_real_roots(0.0, 0.0, 0.0, 1.0);
  EXPECT_TRUE(AreBothNaN(roots));
}

GTEST_TEST(CubicRealRootsTest, IdenticallyZeroPolynomial) {
  // a = b = c = d = 0 -> all points are roots, first is 0, rest are NaN.
  const std::array<double, 3> roots = cubic_real_roots(0.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_TRUE(isnan(roots[1]));
  EXPECT_TRUE(isnan(roots[2]));
}

GTEST_TEST(CubicRealRootsTest, LinearDegenerateCase) {
  // a = b = 0, c != 0 -> linear equation cx + d = 0.
  const std::array<double, 3> roots = cubic_real_roots(0.0, 0.0, 2.0, -6.0);
  // 2x - 6 = 0 -> x = 3.
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_EQ(roots[0], 3.0);
  EXPECT_TRUE(isnan(roots[1]));
  EXPECT_TRUE(isnan(roots[2]));
}

GTEST_TEST(CubicRealRootsTest, QuadraticDegenerateCase) {
  // a = 0, b != 0 -> quadratic case handled by quadratic_real_roots.
  // x^2 - 5x + 6 = 0 -> roots 2 and 3.
  const std::array<double, 3> roots = cubic_real_roots(0.0, 1.0, -5.0, 6.0);
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  EXPECT_TRUE(isnan(roots[2]));
  EXPECT_NEAR(roots[0], 2.0, kTol);
  EXPECT_NEAR(roots[1], 3.0, kTol);
}

GTEST_TEST(CubicRealRootsTest, CubicWithZeroConstantAndNoAdditionalRealRoots) {
  // d = 0, a != 0 -> root at 0 plus roots of quadratic_real_roots(a, b, c).
  // Use a quadratic with no real roots: x^2 + x + 1.
  // Cubic: x * (x^2 + x + 1) = x^3 + x^2 + x -> roots: 0 and complex pair.
  const std::array<double, 3> roots = cubic_real_roots(1.0, 1.0, 1.0, 0.0);
  // Exactly one real root at 0, others NaN.
  EXPECT_EQ(roots[0], 0.0);
  EXPECT_TRUE(isnan(roots[1]));
  EXPECT_TRUE(isnan(roots[2]));
}

GTEST_TEST(CubicRealRootsTest, CubicWithZeroConstantAndThreeRealRoots) {
  // d = 0, a != 0, quadratic has two real roots.
  // x * (x^2 - 5x + 6) = x^3 - 5x^2 + 6x -> roots 0, 2, 3.
  const std::array<double, 3> roots = cubic_real_roots(1.0, -5.0, 6.0, 0.0);
  // This path should sort the roots when both quadratic roots are finite.
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  ASSERT_TRUE(isfinite(roots[2]));
  EXPECT_NEAR(roots[0], 0.0, kTol);
  EXPECT_NEAR(roots[1], 2.0, kTol);
  EXPECT_NEAR(roots[2], 3.0, kTol);
}

GTEST_TEST(CubicRealRootsTest, ThreeDistinctRealRootsGeneralCase) {
  using std::max;
  using std::abs;
  const std::array<double, 3> expected_roots = {-3.45, 0.0678, 0.12};

  const double a = 1.0;
  const double b = -expected_roots[0] - expected_roots[1] - expected_roots[2];
  const double c = expected_roots[0] * expected_roots[1] +
                   expected_roots[1] * expected_roots[2] +
                   expected_roots[2] * expected_roots[0];
  const double d = -expected_roots[0] * expected_roots[1] * expected_roots[2];
  const std::array<double, 3> roots = cubic_real_roots(a, b, c, d);

  // Should return three real roots, sorted.
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  ASSERT_TRUE(isfinite(roots[2]));
  EXPECT_NEAR(roots[0], expected_roots[0], kTol);
  EXPECT_NEAR(roots[1], expected_roots[1], kTol);
  EXPECT_NEAR(roots[2], expected_roots[2], kTol);

  const double expected_residual =
      kTol * (4 * abs(a) + 3 * abs(b) + 2 * abs(c) + abs(d));

  for (double r : roots) {
    const double f = ((a * r + b) * r + c) * r + d;
    EXPECT_NEAR(f, 0.0, expected_residual);
  }
}

GTEST_TEST(CubicRealRootsTest, OneRealRootGeneralCase) {
  // x^3 - x + 1 = 0 has one real root (near -0.682...).
  const std::array<double, 3> roots = cubic_real_roots(1.0, 0.0, -1.0, 1.0);
  ASSERT_TRUE(isfinite(roots[0]));
  EXPECT_TRUE(isnan(roots[1]));
  EXPECT_TRUE(isnan(roots[2]));

  // Check the root satisfies f(x) ≈ 0.
  const double r = roots[0];
  const double f = r * (r * r - 1.0) + 1.0;
  EXPECT_NEAR(f, 0.0, kTol);
}

GTEST_TEST(CubicRealRootsTest, DoubleRootSpecialCase) {
  // (x - 1)^2 (x - 2) = x^3 - 4x^2 + 5x - 2.
  // Double root at x = 1, simple root at 2.
  const std::array<double, 3> roots = cubic_real_roots(1.0, -4.0, 5.0, -2.0);

  // Special-case branch should detect double root and sort.
  ASSERT_TRUE(isfinite(roots[0]));
  ASSERT_TRUE(isfinite(roots[1]));
  ASSERT_TRUE(isfinite(roots[2]));
  EXPECT_NEAR(roots[0], 1.0, kTol);
  EXPECT_NEAR(roots[1], 1.0, kTol);
  EXPECT_NEAR(roots[2], 2.0, kTol);

  for (double r : roots) {
    const double f = ((r - 4.0) * r + 5.0) * r - 2.0;
    EXPECT_NEAR(f, 0.0, kTol);
  }
}

GTEST_TEST(CubicRealRootsTest, ScalingDoesNotChangeRoots) {
  // Use a polynomial with moderate coefficients and scale them.
  // f(x) = x^3 - 3x + 2 = (x - 1)^2 (x + 2)
  const double a = 1.0;
  const double b = 0.0;
  const double c = -3.0;
  const double d = 2.0;

  const double s = 123.456;

  const std::array<double, 3> roots = cubic_real_roots(a, b, c, d);
  const auto scaled_roots =
      cubic_real_roots(s*a, s*b, s*c, s*d);


  EXPECT_NEAR(roots[0], scaled_roots[0], kTol);
  EXPECT_NEAR(roots[1], scaled_roots[1], kTol);
  EXPECT_NEAR(roots[2], scaled_roots[2], kTol);
}

GTEST_TEST(CubicRealRootsTest, NonFiniteCoefficientsThrows) {
  const double inf = std::numeric_limits<double>::infinity();
  const double NaN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(cubic_real_roots(inf, 1.0, 1.0, 1.0),
               std::runtime_error);  // non-finite a
  EXPECT_THROW(cubic_real_roots(1.0, inf, 1.0, 1.0),
               std::runtime_error);  // non-finite b
  EXPECT_THROW(cubic_real_roots(1.0, 1.0, inf, 1.0),
               std::runtime_error);  // non-finite c
  EXPECT_THROW(cubic_real_roots(1.0, 1.0, 1.0, inf),
               std::runtime_error);  // non-finite d
  EXPECT_THROW(cubic_real_roots(NaN, 1.0, 1.0, 1.0),
               std::runtime_error);  // non-finite a
  EXPECT_THROW(cubic_real_roots(1.0, NaN, 1.0, 1.0),
               std::runtime_error);  // non-finite b
  EXPECT_THROW(cubic_real_roots(1.0, 1.0, NaN, 1.0),
               std::runtime_error);  // non-finite c
  EXPECT_THROW(cubic_real_roots(1.0, 1.0, 1.0, NaN),
               std::runtime_error);  // non-finite d
}

}  // namespace
}  // namespace math
}  // namespace drake
