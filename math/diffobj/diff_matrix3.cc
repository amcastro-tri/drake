#include "math/diffobj/diff_matrix3.h"

namespace drake {
namespace math {
namespace diffobj {
namespace internal {

bool IsNearlyEqualTo(const Eigen::Matrix3d& lhs, const Eigen::Matrix3d& rhs,
                     double tolerance) {
  return (rhs - lhs).lpNorm<Eigen::Infinity>() < tolerance;
}

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake

template class ::drake::math::diffobj::internal::DiffMatrix3<
    ::drake::math::diffobj::internal::DenseDerivatives>;
