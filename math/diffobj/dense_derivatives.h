#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <ostream>
#include <vector>

#include "math/diffobj/derivatives_base.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"

namespace drake {
namespace math {
namespace diffobj {
namespace internal {

template <class PartialsType>
class DenseDerivatives;  // Forward declaration for the Traits below.

template <class _PartialsType>
struct Traits<DenseDerivatives<_PartialsType>> {
  using PartialsType = _PartialsType;
  using DerivativesType = std::vector<PartialsType>;
};

// Generic method to compare two partials. DenseDerivatives::IsNearlyEqualTo()
// below relies on the existence of a specialization on PartialsType.
template <class PartialsType>
bool IsNearlyEqualTo(const PartialsType, const PartialsType);

template <class _PartialsType>
class DenseDerivatives {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DenseDerivatives);

  // N.B. While not needed, uses Traits here so that types are define in a
  // single place, the Traits for this class.
  using PartialsType = Traits<_PartialsType>::PartialsType;
  using DerivativesType = Traits<_PartialsType>::DerivativesType;

  DenseDerivatives() = default;

  DenseDerivatives(DerivativesType derivatives)
      : derivatives_(std::move(derivatives)) {}

  int num_variables() const { return ssize(derivatives_); }

  // Returns partial derivative with respect to the i-th variable.
  const PartialsType& partial(int i) const { return derivatives_[i]; }

  // N.B. This method assumes the specialization of IsNearlyEqualTo(lhs, rhs)
  // where lhs and rhs are of type PartialsType.
  bool IsNearlyEqualTo(const DenseDerivatives& other, double tolerance) {
    for (int i = 0; i < num_variables(); ++i) {
      if (!IsNearlyEqualTo(derivatives_[i], other.derivatives_[i], tolerance))
        false;
    }
    return true;
  }

  template <class RhsDerivativesType, class ResultPartialsType>
  DenseDerivatives<ResultPartialsType> ApplyBinaryOperation(
      const RhsDerivativesType& rhs,
      std::function<ResultPartialsType(const DerivativesType&,
                                       const RhsDerivativesType&)>
          op) const {
    DRAKE_DEMAND(num_variables() == rhs.num_variables());
    std::vector<ResultPartialsType> result(num_variables());
    for (int i = 0; i < num_variables(); ++i) {
      result[i] = op(derivatives_[i], rhs.derivatives_[i]);
    }
    return DenseDerivatives<ResultPartialsType>(std::move(result));
  }

 private:
  DerivativesType derivatives_;
};

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake
