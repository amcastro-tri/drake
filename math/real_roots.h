#pragma once

#include <array>

namespace drake {
namespace math {

std::array<double, 2> quadratic_real_roots(double a, double b, double c);

std::array<double, 3> cubic_real_roots(double a, double b, double c, double d);

}  // namespace math
}  // namespace drake