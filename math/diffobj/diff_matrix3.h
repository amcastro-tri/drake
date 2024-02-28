#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <ostream>
#include <vector>

#include "math/diffobj/dense_derivatives.h"
#include "math/diffobj/diff_object.h"
#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"

namespace drake {
namespace math {
namespace diffobj {
namespace internal {

// Generic method to compare two partials. DiffMatrix3::IsNearlyEqualTo()
// below relies on the existence of a specialization on PartialsType.
// template <class PartialsType>
// bool IsNearlyEqualTo(const PartialsType, const PartialsType);

template <template <class> class DerivativesType>
class DiffMatrix3;  // forward declaration for Traits below.

template <template <class> class DerivativesContainerType>
struct Traits<DiffMatrix3<DerivativesContainerType>> {
  using ValueType = Eigen::Matrix3d;
  using PartialsType = Eigen::Matrix3d;
  using DerivativesType = DerivativesContainerType<PartialsType>;
};

// Specialization to to compare both values and derivatives.
// It returns true if the max norm of the difference is strictly smaller than
// tolerance.
bool IsNearlyEqualTo(const Eigen::Matrix3d& lhs, const Eigen::Matrix3d& rhs,
                     double tolerance) {
  return (rhs - lhs).lpNorm<Eigen::Infinity>() < tolerance;
}

template <template <class> class DerivativesContainerType>
class DiffMatrix3 : public DiffObject<DiffMatrix3<DerivativesContainerType>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiffMatrix3);

  // N.B. While not needed, uses Traits here so that types are define in a
  // single place, the Traits for this class.
  using ValueType = Traits<DiffMatrix3>::ValueType;
  using PartialsType = Traits<DiffMatrix3>::PartialsType;
  using DerivativesType = Traits<DiffMatrix3>::DerivativesType;
  using Base = DiffObject<DiffMatrix3<DerivativesContainerType>>;

  using Base::derivatives;
  using Base::num_variables;
  using Base::value;

  DiffMatrix3() = default;

  DiffMatrix3(ValueType v, DerivativesType d)
      : value_(std::move(v)), derivatives_(std::move(d)) {}

 private:
  ValueType value_;
  DerivativesType derivatives_;
};

using Matrix3WithDenseDerivatives = DiffMatrix3<DenseDerivatives>;

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake
