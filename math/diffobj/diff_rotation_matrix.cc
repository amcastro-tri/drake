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
DiffRotationMatrix<DerivativesContainerType>::DiffRotationMatrix(
    const Matrix3<AutoDiffXd>& M) {
  *this = MakeFromAutoDiffXd(M);
}

template <template <class> class DerivativesContainerType>
DiffRotationMatrix<DerivativesContainerType>
DiffRotationMatrix<DerivativesContainerType>::MakeZRotation(
    const AutoDiffXd& theta) {
  using std::cos;
  using std::sin;

  const double c = cos(theta.value()), s = sin(theta.value());
  Eigen::Matrix3d R_AB = Eigen::Matrix3d::Zero();
  R_AB(0, 0) = c, R_AB(0, 1) = -s;
  R_AB(1, 0) = s, R_AB(1, 1) = c;

  const int num_variables = theta.derivatives().size();
  DerivativesType derivatives(num_variables);  
  if (num_variables > 0) {
    const Eigen::Vector3d minus_w = -Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d wR = R_AB.colwise().cross(minus_w);
    for (int i = 0; i < num_variables; ++i) {
      if (theta.derivatives()[i] != 0.0) {
        derivatives.SetPartial(i, wR * theta.derivatives()[i]);
      }
    }
  }
  return DiffRotationMatrix(std::move(R_AB), std::move(derivatives));
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
Vector3<AutoDiffXd> DiffRotationMatrix<DerivativesContainerType>::operator*(
    const Vector3<AutoDiffXd>& v) const {
  return ToAutoDiffXd() * v;
}

template <template <class> class DerivativesContainerType>
DiffRotationMatrix<DerivativesContainerType>
DiffRotationMatrix<DerivativesContainerType>::operator*(
    const DiffRotationMatrix<DerivativesContainerType>& rhs) const {
  struct MultiplyOperation {
    using LhsType = DiffRotationMatrix;
    using RhsType = DiffRotationMatrix;
    using ResultType = DiffRotationMatrix;

    using LhsValueType = typename LhsType::ValueType;
    using LhsPartialsType = typename LhsType::PartialsType;
    using RhsValueType = typename RhsType::ValueType;
    using RhsPartialsType = typename RhsType::PartialsType;
    using ResultPartialsType = typename ResultType::PartialsType;
    using ResultValueType = typename ResultType::ValueType;

    struct DataType {
      const LhsValueType& lhs_value;
      const RhsValueType& rhs_value;
      ResultValueType value;
    };

    // In this case, we choose to store the value as data.
    static DataType CalcData(const DiffRotationMatrix& lhs,
                             const DiffRotationMatrix& rhs) {
      return DataType{lhs.value(), rhs.value(), lhs.value() * rhs.value()};
    }

    static ResultValueType CalcValue(const DataType& data, const LhsValueType&,
                                     const RhsValueType&) {
      // Return the already computed value.
      return data.value;
    }

    static ResultPartialsType CalcPartial(const DataType& data,
                                          const LhsPartialsType* lhs_partial,
                                          const RhsPartialsType* rhs_partial) {
      DRAKE_ASSERT(lhs_partial || rhs_partial);
      if (lhs_partial && rhs_partial) {
        // Both partials are non-zero.
        return (*lhs_partial) * data.rhs_value +
               data.lhs_value * (*rhs_partial);
      } else if (lhs_partial) {
        // rhs partial is zero.
        return (*lhs_partial) * data.rhs_value;
      } else {
        // lhs partial is zero.
        return data.lhs_value * (*rhs_partial);
      }
    }
  };

  return this->template ApplyBinaryOperation<MultiplyOperation>(rhs);
}

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake

template class ::drake::math::diffobj::internal::DiffRotationMatrix<
    ::drake::math::diffobj::internal::DenseDerivatives>;
template class ::drake::math::diffobj::internal::DiffRotationMatrix<
    ::drake::math::diffobj::internal::OptionalDerivatives>;
template class ::drake::math::diffobj::internal::DiffRotationMatrix<
    ::drake::math::diffobj::internal::SparseDerivatives>;
