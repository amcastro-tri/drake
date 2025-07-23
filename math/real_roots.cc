#include "drake/math/real_roots.h"
#include "drake/common/drake_throw.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace drake {
namespace math {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double sqrt3 = std::numbers::sqrt3;

std::array<double, 2> quadratic_real_roots(double a, double b, double c) {
  using std::copysign;
  using std::sqrt;

  DRAKE_THROW_UNLESS(std::isfinite(a));
  DRAKE_THROW_UNLESS(std::isfinite(b));
  DRAKE_THROW_UNLESS(std::isfinite(c));

  // Handle degenerate cases.
  if (a == 0) {
    // Non-zero constant polynomial has no roots.
    if (b == 0 && c != 0) {
      return {NaN, NaN};
    } else if (b == 0 && c == 0) {
      // Technically infinite roots, so we'll just return one valid root.
      return {0.0, NaN};
    }
    // Linear case.
    return {-c / b, NaN};
  }
  if (b == 0) {
    // Double root at x = 0.
    if (c == 0) {
      return {0.0, 0.0};
    }
    const double x0_sq = -c / a;
    // No real roots.
    if (x0_sq < 0) {
      return {NaN, NaN};
    }
    const double x0 = sqrt(x0_sq);
    return {-x0, x0};
  }
  if (c == 0) {
    const double x0 = -b / a;
    if (x0 < 0) {
      return {x0, 0.0};
    } else {
      return {0.0, x0};
    }
  }

  // General case.
  const double discriminant = b * b - 4.0 * a * c;

  // No real roots.
  if (discriminant < 0) {
    return {NaN, NaN};
  }
  // Avoid catastrophic cancellation.
  const double q = -0.5 * (b + copysign(sqrt(discriminant), b));
  const double x0 = q / a;
  const double x1 = c / q;
  if (x0 < x1) {
    return {x0, x1};
  }
  return {x1, x0};
}

// Solves for the real roots of ax^3 + bx^2 + cx + d = 0.
// Adapted from:
//   https://github.com/boostorg/math/blob/develop/include/boost/math/tools/cubic_roots.hpp.
// Follows Numerical Recipes, Chapter 5, section 6.
// Adds scaling to avoid overflow/underflow. See line 1080 of RPOLY:
//   Jenkins, Michael A. "Algorithm 493: Zeros of a real polynomial [c2]." ACM
//   Transactions on Mathematical Software (TOMS) 1.2 (1975): 178-189.
std::array<double, 3> cubic_real_roots(double a, double b, double c, double d) {
  using std::abs;
  using std::acos;
  using std::cbrt;
  using std::clamp;
  using std::cos;
  using std::isfinite;
  using std::max;
  using std::sqrt;

  DRAKE_THROW_UNLESS(isfinite(a));
  DRAKE_THROW_UNLESS(isfinite(b));
  DRAKE_THROW_UNLESS(isfinite(c));
  DRAKE_THROW_UNLESS(isfinite(d));

  std::array<double, 3> roots = {NaN, NaN, NaN};

  // Scale the coefficients by an exact power of 2 to control
  // overflow/underflow without losing precision.
  const double m = max(max(abs(a), abs(b)), max(abs(c), abs(d)));
  const int e = std::ilogbl(m);
  double s = std::scalbn(1.0, -e);
  a *= s;
  b *= s;
  c *= s;
  d *= s;

  // Handle degenerate cases.
  if (a == 0) {
    if (b == 0) {
      if (c == 0) {
        if (d != 0) {
          return roots;
        }
        // Technically infinite roots, so we'll just return one valid root.
        roots[0] = 0;
        roots[1] = NaN;
        roots[2] = NaN;
        return roots;
      }
      roots[0] = -d / c;
      return roots;
    }
    auto [x0, x1] = quadratic_real_roots(b, c, d);
    roots[0] = x0;
    roots[1] = x1;
    return roots;
  }
  if (d == 0) {
    auto [x0, x1] = quadratic_real_roots(a, b, c);
    roots[0] = 0;
    roots[1] = x0;
    roots[2] = x1;
    if (isfinite(x0)) {
      if (isfinite(x1)) {
        std::sort(roots.begin(), roots.end());
      } else if (x0 < 0) {
        std::swap(roots[0], roots[1]);
      }
    }
    return roots;
  }

  // General case;
  const double p = b / a;
  const double q = c / a;
  const double r = d / a;
  const double Q = (p * p - 3 * q) / 9;
  const double R = (2 * p * p * p - 9 * p * q + 27 * r) / 54;
  if (R * R < Q * Q * Q) {
    const double rtQ = sqrt(Q);
    const double theta = acos(clamp(R / (Q * rtQ), -1.0, 1.0)) / 3;
    const double st = sin(theta);
    const double ct = cos(theta);
    roots[0] = -2 * rtQ * ct - p / 3;
    roots[1] = -rtQ * (-ct + sqrt3 * st) - p / 3;
    roots[2] = rtQ * (ct + sqrt3 * st) - p / 3;
    std::sort(roots.begin(), roots.end());
  } else {
    const double arg = R * R - Q * Q * Q;
    const double A = (R >= 0 ? -1 : 1) * cbrt(abs(R) + sqrt(arg));
    double B = 0;
    if (A != 0) {
      B = Q / A;
    }
    roots[0] = A + B - p / 3;
    // Special case: double real root.
    if (A == B || arg == 0) {
      roots[1] = -A - p / 3;
      roots[2] = -A - p / 3;
      std::sort(roots.begin(), roots.end());
    }
  }
  // Root polishing. If possible, do one iteration of Halley's method or
  // Newton's method.
  for (auto& x : roots) {
    // Skip any NaN values.
    if(!isfinite(r)) continue;

    const double f = ((a * x + b) * x + c) * x + d;
    const double df = (3 * a * x + 2 * b) * x + c;
    if (df != 0) {
      const double d2f = 6 * a * x + 2 * b;
      const double denom = 2 * df * df - f * d2f;
      if (denom != 0) {
        // Halley's method.
        x -= 2 * f * df / denom;
      } else {
        // Newton's method.
        x -= f / df;
      }
    }
  }
  return roots;
}

}  // namespace math
}  // namespace drake