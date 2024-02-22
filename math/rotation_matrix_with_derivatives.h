/** @file
Internal representation for a rotation matrix with derivatives. */

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <ostream>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/math/object_with_derivatives.h"

// #include "drake/common/symbolic/expression.h"
// #include "drake/math/autodiff_gradient.h"

namespace drake {
namespace math {
namespace internal {

// Options are:
//   - Dense or sparse derivatives. True or False should suffice.
//   - Full Matrix3d or SkewSymmetricMatrix (or Vector3) for the partials.

// Forward declaration for the traits below.
class RotationMatrixWithDerivatives;

template <>
struct Traits<RotationMatrixWithDerivatives> {
  using ValueType = Eigen::Matrix3d;
  // TODO: change to SkewSymmetricMatrix
  using PartialsType = Eigen::Matrix3d;
};

class RotationMatrixWithDerivatives
    : public ObjectWithDerivatives<RotationMatrixWithDerivatives> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationMatrixWithDerivatives);

  using MyTraits = Traits<RotationMatrixWithDerivatives>;
  using ValueType = MyTraits::ValueType;
  using PartialsType = MyTraits::PartialsType;
  using Base = ObjectWithDerivatives<RotationMatrixWithDerivatives>;

  // Uninitialized rotation with no-derivatives.
  RotationMatrixWithDerivatives() = default;

  // Constructs a constant value with no derivatives.
  explicit RotationMatrixWithDerivatives(Eigen::Matrix3d value)
      : Base(std::move(value)) {}

  RotationMatrixWithDerivatives(ValueType value,
                                std::vector<PartialsType> derivatives)
      : Base(std::move(value), std::move(derivatives)) {}

  static RotationMatrixWithDerivatives Identity() {
    return RotationMatrixWithDerivatives(
        ValueType(Eigen::Matrix3d::Identity()));
  }

  static RotationMatrixWithDerivatives MakeZRotation(const AutoDiffXd& theta);

  // WARNING: For efficiency reasons, avoid using this constructor at all costs.
  // It is only provided for convenience.
  explicit RotationMatrixWithDerivatives(const Matrix3<AutoDiffXd>& M);

  RotationMatrixWithDerivatives operator*(
      const RotationMatrixWithDerivatives& rhs) const {
    return RotationMatrixWithDerivatives(MultiplyOperator::Calc(*this, rhs));
  }
  
  RotationMatrixWithDerivatives transpose() const;

  bool IsExactlyIdentity() const;  

  // WARNING: For efficiency reasons, avoid using this function at all costs.
  // It is only provided for convenience.
  // TODO: REMOVE THIS!!!
  Vector3<AutoDiffXd> operator*(const Vector3<AutoDiffXd>& v_B) const;

  // WARNING: For efficiency reasons, avoid using this function at all costs.
  // It is only provided for convenience.
  Matrix3<AutoDiffXd> ToAutoDiffXd() const;

  // WARNING: For efficiency reasons, avoid using this function at all costs.
  // It is only provided for convenience.
  void operator=(const MatrixX<AutoDiffXd>& M) {
    *this = RotationMatrixWithDerivatives(Matrix3<AutoDiffXd>(M));
  }

 private:
  struct MultiplyOperation {
    // Types involed in the operation.
    using LhsType = RotationMatrixWithDerivatives;
    using RhsType = RotationMatrixWithDerivatives;
    using ResultType = RotationMatrixWithDerivatives;

    // Aliases.
    using LhsValueType = LhsType::ValueType;
    using RhsValueType = RhsType::ValueType;
    using ResultValueType = ResultType::ValueType;
    using LhsPartialType = LhsType::PartialsType;
    using RhsPartialType = RhsType::PartialsType;
    using ResultPartialType = ResultType::PartialsType;

    static ResultValueType CalcValue(const LhsValueType& lhs,
                                     const RhsValueType& rhs) {
      return lhs * rhs;
    }

    static ResultPartialType CalcPartial(const ResultValueType& result,
                                         const LhsValueType& lhs,
                                         const LhsPartialType& lhs_partial,
                                         const RhsValueType& rhs,
                                         const RhsPartialType& rhs_partial) {
      unused(result);
      return lhs_partial * rhs + lhs * rhs_partial;
    }
  };
  using MultiplyOperator =
      ObjectWithDerivativesBinaryOperation<MultiplyOperation>;
};

bool IsNearlyEqualTo(const RotationMatrixWithDerivatives& m1,
                     const RotationMatrixWithDerivatives& m2,
                     double tolerance);

bool operator==(const RotationMatrixWithDerivatives& m1,
                const RotationMatrixWithDerivatives& m2);

}  // namespace internal
}  // namespace math
}  // namespace drake
