#include "drake/multibody/contact_solvers/newton_with_bisection.h"

#include <functional>
#include <iostream>
#include <tuple>
#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

std::pair<double, int>
DoNewtonWithBisectionFallback(
    const std::function<std::pair<double, double>(double)>& function,
    Bracket bracket, double x_guess, double x_tolerance, double f_tolerance,
    int max_iterations) {
  using std::abs;
  using std::swap;
  // Pre-conditions on the bracket.
  DRAKE_THROW_UNLESS(bracket.inside(x_guess));

  // Pre-conditions on the algorithm's parameters.
  DRAKE_THROW_UNLESS(x_tolerance > 0);
  DRAKE_THROW_UNLESS(f_tolerance > 0);
  DRAKE_THROW_UNLESS(max_iterations > 0);

  if (abs(bracket.f_lower()) < f_tolerance)
    return std::make_pair(bracket.x_lower(), 0);

  if (abs(bracket.f_upper()) < f_tolerance)
    return std::make_pair(bracket.x_upper(), 0);

  double root = x_guess;  // Initialize to user supplied guess.
  double minus_dx = bracket.x_lower() - bracket.x_upper();
  double minus_dx_previous = minus_dx;
  double f, df;
  std::tie(f, df) = function(root);  // First evaluation.
  if (abs(f) < f_tolerance) return std::make_pair(root, 1);
  double f_previous = f;

  // Helper to perform a bisection update. It returns the pair (root, -dx).
  auto do_bisection = [&bracket]() {
    const double dx_negative = 0.5 * (bracket.x_lower() - bracket.x_upper());
    // N.B. This way of updating the root will lead to root == x_lower if
    // the value of minus_dx is insignificant compared to x_lower when using
    // floating point precision. This fact is used in the termination check
    // below to exit whenever a user specifies f_tolerance = 0.
    const double x = bracket.x_lower() - dx_negative;
    return std::make_pair(x, dx_negative);
  };

  // Helper to perform a Newton update. It returns the pair (root, -dx).
  auto do_newton = [&f, &df, &root]() {
    const double dx_negative = f / df;
    double x = root;
    // N.B. x will not change if dx_negative is negligible within machine
    // precision.
    x -= dx_negative;
    return std::make_pair(x, dx_negative);
  };

#if 0
  struct HistData {
    double x;
    double f;
    double df;
  };
  std::vector<HistData> hist;
  hist.push_back(HistData{root, f, df});
#endif  

  for (int num_evaluations = 1; num_evaluations <= max_iterations;
       ++num_evaluations) {
    // N.B. Notice this check is always true for df = 0 (and f != 0 since we
    // ruled that case out above). Therefore Newton is only called when df != 0,
    // and the search direction is well defined.
    // N.B. This check is based on the check used within method rtsafe from
    // Numerical Recipes. While rtsafe uses dx from the previous to last
    // iteration, here we use dx from precisely the previous iteration. We found
    // this to save a few iterations when compared to rtsafe.
    // N.B. One way to think about this: if we assume 0 ≈ |fᵏ| << |fᵏ⁻¹| (this
    // would be the case when Newton is converging quadratically), then we can
    // estimate fᵏ⁻¹ from values at the last iteration as fᵏ⁻¹ ≈ fᵏ + dx⋅f'ᵏ ≈
    // dx⋅f'ᵏ. Therefore the inequality below is an approximation for |2⋅fᵏ| >
    // |fᵏ⁻¹|. That is, we use Newton's method when |fᵏ| < |fᵏ⁻¹|/2. Otherwise
    // we use bisection which guarantees convergence, though linearly.
    const bool newton_is_slow = 2.0 * abs(f) > abs(minus_dx_previous * df);

    minus_dx_previous = minus_dx;
    if (newton_is_slow) {
      std::tie(root, minus_dx) = do_bisection();
      DRAKE_LOGGER_DEBUG("Bisect. k = {:d}.", num_evaluations);
    } else {
      std::tie(root, minus_dx) = do_newton();
      if (bracket.inside(root)) {
        DRAKE_LOGGER_DEBUG("Newton. k = {:d}.", num_evaluations);
      } else {
        std::tie(root, minus_dx) = do_bisection();
        DRAKE_LOGGER_DEBUG("Bisect. k = {:d}.", num_evaluations);
      }
    }    

    //if (abs(minus_dx) < f_tolerance)
    //  return std::make_pair(root, num_evaluations);

    // The one evaluation per iteration.
    f_previous = f;
    std::tie(f, df) = function(root);
    //hist.push_back(HistData{root, f, df});

    DRAKE_LOGGER_DEBUG(
        "x = {:10.4g}. [x_lower, x_upper] = [{:10.4g}, "
        "{:10.4g}]. dx = {:10.4g}. f = {:10.4g}. dfdx = {:10.4g}.",
        root, bracket.x_lower(), bracket.x_upper(), -minus_dx, f, df);

    if (abs(f) < f_tolerance || abs(minus_dx) < x_tolerance * root)
      return std::make_pair(root, num_evaluations);

    // Update the bracket around root to guarantee that there exist a root
    // within the interval [x_lower, x_upper].
    bracket.Update(root, f);
  }

#if 0
  std::cout << "History:\n";
  for (auto&d : hist) {
    std::cout << fmt::format("{} {} {}\n", d.x, d.f, d.df);
  }
#endif  

  // If here, then DoNewtonWithBisectionFallback did not converge.
  // This will happen for instance when the maximum number of iterations is too
  // small.
  throw std::runtime_error(fmt::format(
      "NewtonWithBisectionFallback did not converge.\n"
      "|dx| = {}. |x_upper - x_lower| = {}. x = {}. f = {}.",
      abs(minus_dx), abs(bracket.x_upper() - bracket.x_lower()), root, f));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
