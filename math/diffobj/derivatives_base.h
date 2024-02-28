#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <ostream>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"

namespace drake {
namespace math {
namespace diffobj {
namespace internal {

// TODO: move this somewhere more appropriate.
template <class TypeWithTraits>
struct Traits {};

template <class Derived>
class DerivativesBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DerivativesBase);

  using PartialsType = Traits<Derived>::PartialsType;
  using DerivativesType = Traits<Derived>::DerivativesType;

  DerivativesBase() = default;

  const Derived& derived() const { return *static_cast<const Derived*>(this); }
  Derived& mutable_derived() { return *static_cast<Derived*>(this); }

  int num_variables() const { return derived().num_variables(); }

  bool IsNearlyEqualTo(const Derived& other, double tolerance) {
    return derived().IsNearlyEqualTo(other, tolerance);
  }

  template <class RhsDerivativesType, class ResultPartialsType>
  Derived ApplyBinaryOperation(
      const RhsDerivativesType& rhs,
      std::function<ResultPartialsType(const DerivativesType&,
                                       const RhsDerivativesType&)>
          op) const {
    return derived().ApplyBinaryOperation(rhs.derived(), op);
  }
};

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake
