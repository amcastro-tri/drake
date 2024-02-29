#include "math/diffobj/diff_rotation_matrix.h"

using Eigen::Matrix3d;

namespace drake {
namespace math {
namespace diffobj {
namespace internal {

bool AreNearlyEqual(const Eigen::Matrix3d& lhs, const Eigen::Matrix3d& rhs,
                    double tolerance) {
  return (rhs - lhs).lpNorm<Eigen::Infinity>() < tolerance;
}

template <template <class> class DerivativesContainerType>
DiffRotationMatrix<DerivativesContainerType>
DiffRotationMatrix<DerivativesContainerType>::transpose() const {
  struct TransposeOperation {
    using PartialsType = typename DiffRotationMatrix::PartialsType;
    using ResultPartialsType = typename DiffRotationMatrix::PartialsType;

    static ResultPartialsType Calc(const PartialsType& p) {
        return p.transpose();
    }
  };

  Matrix3d v = value().transpose();
  DerivativesType d =
      derivatives().template ApplyUnaryOperation<TransposeOperation>();
  return DiffRotationMatrix(std::move(v), std::move(d));
}

template <template <class> class DerivativesContainerType>
bool DiffRotationMatrix<DerivativesContainerType>::IsExactlyIdentity() const {
  // Verify that value equals the identiy matrix.
  if (value() != Matrix3d::Identity()) {
    return false;
  }
  auto partial_is_zero = [](const Matrix3d& partial) {
    return partial.isZero();
  };

  // Verify all partials are zero.
  if (!derivatives().AllOf(partial_is_zero)) return false;

  return true;
}

template <template <class> class DerivativesContainerType>
bool DiffRotationMatrix<DerivativesContainerType>::IsNearlyEqualTo(
    const DiffRotationMatrix<DerivativesContainerType>& other,
    double tolerance) const {
  if (!AreNearlyEqual(value(), other.value(), tolerance)) return false;

  auto nearly_equal_partials = [tolerance](const Eigen::Matrix3d& lhs,
                                           const Eigen::Matrix3d& rhs) {
    return AreNearlyEqual(lhs, rhs, tolerance);
  };

  if (!derivatives().AllOf(other.derivatives(), nearly_equal_partials))
    return false;

  return true;
}

template <template <class> class DerivativesContainerType>
Vector3<AutoDiffXd> DiffRotationMatrix<DerivativesContainerType>::
    DiffRotationMatrix<DerivativesContainerType>::operator*(
        const Vector3<AutoDiffXd>& v) const {
  return ToAutoDiffXd() * v;
}

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake

template class ::drake::math::diffobj::internal::DiffRotationMatrix<
    ::drake::math::diffobj::internal::DenseDerivatives>;
template class ::drake::math::diffobj::internal::DiffRotationMatrix<
    ::drake::math::diffobj::internal::OptionalDerivatives>;    

