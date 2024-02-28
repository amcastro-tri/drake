#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <ostream>
#include <vector>

#include "math/diffobj/dense_derivatives.h"
#include "math/diffobj/optional_derivatives.h"
#include "math/diffobj/diff_object.h"
#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace math {
namespace diffobj {
namespace internal {

// Generic method to compare two partials. DiffRotationMatrix::IsNearlyEqualTo()
// below relies on the existence of a specialization on PartialsType.
// template <class PartialsType>
// bool IsNearlyEqualTo(const PartialsType, const PartialsType);

template <template <class> class DerivativesType>
class DiffRotationMatrix;  // forward declaration for Traits below.

template <template <class> class _DerivativesContainerType>
struct Traits<DiffRotationMatrix<_DerivativesContainerType>> {
  using ValueType = Eigen::Matrix3d;
  using PartialsType = Eigen::Matrix3d;
  template <class P>
  using DerivativesContainerType = _DerivativesContainerType<P>;
};

// Specialization to to compare both values and derivatives.
// It returns true if the max norm of the difference is strictly smaller than
// tolerance.
bool IsNearlyEqualTo(const Eigen::Matrix3d& lhs, const Eigen::Matrix3d& rhs,
                     double tolerance);

template <template <class> class DerivativesContainerType>
class DiffRotationMatrix : public DiffObject<DiffRotationMatrix<DerivativesContainerType>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiffRotationMatrix);

  // N.B. While not needed, uses Traits here so that types are define in a
  // single place, the Traits for this class.
  using ValueType = typename Traits<DiffRotationMatrix>::ValueType;
  using PartialsType = typename Traits<DiffRotationMatrix>::PartialsType;
  using DerivativesType = DerivativesContainerType<PartialsType>;
  using Base = DiffObject<DiffRotationMatrix<DerivativesContainerType>>;

  using Base::derivatives;
  using Base::num_variables;
  using Base::value;
  using Base::ApplyBinaryOperation;

  DiffRotationMatrix() = default;

  DiffRotationMatrix(ValueType v, DerivativesType d)
      : Base(std::move(v), std::move(d)) {}

  // Constructs a constant value with no derivatives.
  explicit DiffRotationMatrix(Eigen::Matrix3d value)
      : Base(std::move(value)) {}    

  static DiffRotationMatrix Identity() {
    return DiffRotationMatrix(
        ValueType(Eigen::Matrix3d::Identity()));
  }

  static DiffRotationMatrix MakeZRotation(const AutoDiffXd& theta);

  DiffRotationMatrix transpose() const;

  bool IsExactlyIdentity() const;

  DiffRotationMatrix operator*(const DiffRotationMatrix& rhs) const {
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
      static DataType CalcData(const DiffRotationMatrix& lhs, const DiffRotationMatrix& rhs) {
        return DataType{lhs.value(), rhs.value(), lhs.value() * rhs.value()};
      }

      static ResultValueType CalcValue(const DataType& data,
                                       const LhsValueType&,
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

  // This allows DiffRotationMatrix to interact with Vector3<AutoDiffXd>.
  // @warning Do NOT use this in production code.   
  Vector3<AutoDiffXd> operator*(const Vector3<AutoDiffXd>& v_B) const;

  // Factory to make a DiffRotationMatrix from a Matrix3 object instantiated on
  // AutoDiffXd.
  // The number of derivatives is assumed to be M.cols(). 
  // All partial derivatives are included, even those that are zero.
  // @warning This is an expensive construtor. Avoid using in production code.
  static DiffRotationMatrix MakeFromAutoDiffXd(const Matrix3<AutoDiffXd>& M);

  // @warning This is an expensive function. Avoid using in production code.
  Matrix3<AutoDiffXd> ToAutoDiffXd() const;
};

template <template <class> class DerivativesContainerType>
DiffRotationMatrix<DerivativesContainerType>
DiffRotationMatrix<DerivativesContainerType>::MakeFromAutoDiffXd(
    const Matrix3<AutoDiffXd>& M) {
  using DM3 = DiffRotationMatrix<DerivativesContainerType>;      
  //using ValueType = typename DM3::ValueType;
  //using PartialsType = typename DM3::PartialsType;
  //using DerivativesType = typename DM3::DerivativesType;
  Matrix3<double> Mv = ExtractValue(M);
  // Each partial is stored in a column of Md.
  Eigen::Matrix<double, 9, Eigen::Dynamic> Md = ExtractGradient(M);

  ValueType value(Mv);  // In this scope we know ValueType = Matrix3d.

  const int num_variables = Md.cols();
  std::vector<PartialsType> partials(num_variables);  
  for (int i = 0; i < num_variables; ++i) {
    partials[i] = Md.col(i).reshaped(3, 3);
  }

  // This constructor makes dense derivatives with num_variables =
  // partials.size().
  DerivativesType derivatives(std::move(partials));

  return DM3(std::move(value), std::move(derivatives));
}

template <template <class> class DerivativesContainerType>
Matrix3<AutoDiffXd> DiffRotationMatrix<DerivativesContainerType>::ToAutoDiffXd()
    const {
  Matrix3<double> Md = value();
  const std::vector<Matrix3<double>> partials =
      derivatives().MakeDenseStdVectorOfPartials();
  Eigen::Matrix<double, 9, Eigen::Dynamic> gradient(9, num_variables());
  for (int i = 0; i < num_variables(); ++i) {
    gradient.col(i) = partials[i].reshaped(9, 1);
  }
  Matrix3<AutoDiffXd> Mad;
  InitializeAutoDiff(Md, gradient, &Mad);
  return Mad;
}

using RotatioMatrixWithDenseDerivatives = DiffRotationMatrix<DenseDerivatives>;
using RotatioMatrixWithOptionalDerivatives = DiffRotationMatrix<OptionalDerivatives>;

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake

extern template class ::drake::math::diffobj::internal::DiffRotationMatrix<
    ::drake::math::diffobj::internal::DenseDerivatives>;
extern template class ::drake::math::diffobj::internal::DiffRotationMatrix<
    ::drake::math::diffobj::internal::OptionalDerivatives>;

