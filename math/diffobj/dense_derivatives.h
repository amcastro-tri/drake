#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <ostream>
#include <vector>

#include "math/diffobj/derivatives_base.h"

#include "drake/common/eigen_types.h"
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

template <class PartialsType>
class DenseDerivatives;  // Forward declaration for the Traits below.

//template <class _PartialsType>
//struct Traits<DenseDerivatives<_PartialsType>> {
//  using PartialsType = _PartialsType;
//};

// Generic method to compare two partials. DenseDerivatives::IsNearlyEqualTo()
// below relies on the existence of a specialization on PartialsType.
template <class PartialsType>
bool IsNearlyEqualTo(const PartialsType, const PartialsType);

template <class _PartialsType>
class DenseDerivatives
    : public DerivativesBase<DenseDerivatives, _PartialsType> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DenseDerivatives);

  using PartialsType = _PartialsType;  
  using Base = DenseDerivatives<PartialsType>;

  DenseDerivatives() = default;

  DenseDerivatives(std::vector<PartialsType> derivatives)
      : derivatives_(std::move(derivatives)) {}

  std::vector<PartialsType> MakeDenseStdVectorOfPartials() const {
    return derivatives_;
  }

  int num_variables() const { return ssize(derivatives_); }

  // Returns partial derivative with respect to the i-th variable.
  const PartialsType& partial(int i) const { return derivatives_[i]; }

  bool IsExactlyZero() const {
    if (num_variables() == 0) return true;
    for (int i = 0; i < num_variables(); ++i) {
      if (!derivatives_[i].isZero(0.0)) return false;
    }
    return true;
  }

  // N.B. This method assumes the specialization of IsNearlyEqualTo(lhs, rhs)
  // where lhs and rhs are of type PartialsType.
  bool IsNearlyEqualTo(const DenseDerivatives& other, double tolerance) {
    for (int i = 0; i < num_variables(); ++i) {
      if (!IsNearlyEqualTo(derivatives_[i], other.derivatives_[i], tolerance))
        return false;
    }
    return true;
  }

  template <class Operation>
  DenseDerivatives<typename Operation::ResultPartialsType> ApplyUnaryOperation()
      const {
    std::vector<typename Operation::ResultPartialsType> result(num_variables());
    for (int i = 0; i < num_variables(); ++i) {
      result[i] = Operation::Calc(derivatives_[i]);
    }
    return DenseDerivatives<typename Operation::ResultPartialsType>(
        std::move(result));
  }

  template <class RhsDerivativesType, class ResultPartialsType>
  DenseDerivatives<ResultPartialsType> ApplyBinaryOperation(
      const RhsDerivativesType& rhs,
      std::function<
          ResultPartialsType(const PartialsType*,
                             const typename RhsDerivativesType::PartialsType*)>
          op) const {
    DRAKE_DEMAND(num_variables() == rhs.num_variables());
    std::vector<ResultPartialsType> result(num_variables());
    for (int i = 0; i < num_variables(); ++i) {
      result[i] = op(&derivatives_[i], &rhs.derivatives_[i]);
    }
    return DenseDerivatives<ResultPartialsType>(std::move(result));
  }

 private:
  std::vector<PartialsType> derivatives_;
};

#if 0
template <class _PartialsType>
std::vector<_PartialsType>
DenseDerivatives<_PartialsType>::MakeDenseStdVectorOfPartials() const {
  return derivatives_;
}
#endif

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake
