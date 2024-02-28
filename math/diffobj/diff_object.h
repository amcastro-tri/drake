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

template <class Derived>
class DiffObject {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiffObject);

  // N.B. While not needed, uses Traits here so that types are define in a
  // single place, the Traits for this class.
  using ValueType = Traits<Derived>::ValueType;
  using DerivativesType = Traits<Derived>::DerivativesType;
  using PartialsType = Traits<DerivativesType>::PartialsType;

  DiffObject() = default;

  DiffObject(ValueType value, DerivativesType derivatives)
      : value_(std::move(value)), derivatives_(std::move(derivatives)) {}

  int num_variables() const { return derivatives_.num_variables(); }

  const ValueType& value() const { return value_; }
  const DerivativesType& derivatives() const { return derivatives_; }

  // N.B. This method assumes the existece of a specialization of
  // IsNearlyEqualTo(lhs, rhs) where lhs and rhs are of type ValueType.
  bool IsNearlyEqualTo(const Derived& other, double tolerance) {
    if (!IsNearlyEqualTo(this->value(), other.value())) return false;
    if (!IsNearlyEqualTo(this->derivatives(), other.derivatives()))
      return false;
    return true;
  }

  template <class RhsDiffObjType, class Operation>
  Derived ApplyBinaryOperation(const RhsDiffObjType& rhs,
                               const Operation& op) const {
    DRAKE_DEMAND(num_variables() == rhs.num_variables());
    static_assert(std::is_same_v<Derived, typename Operation::LhsDiffObjType>);
    static_assert(
        std::is_same_v<RhsDiffObjType, typename Operation::RhsDiffObjType>);
    const typename Operation::DataType data = op.CalcData(*this, rhs);
    const typename Operation::ValueType result_value =
        op.CalcValue(data, value(), rhs.value());
    // TODO: Consider placing this lambda's captured data into a single struct to
    // minimize copying cost.        
    auto partials_calc = [&data, &op](
                             const PartialsType& lhs_partial,
                             const PartialsType& rhs_partial) -> PartialsType {
        return op.CalcPartial(data, lhs_partial, rhs_partial);
    };
    const typename Operation::DerivativesType result_derivatives =
        derivatives().ApplyBinaryOperation(rhs.derivatives(), partials_calc);
    return Derived(std::move(result_value), std::move(result_derivatives));
  }

  const Derived& derived() const { return *static_cast<const Derived*>(this); }
  Derived& mutable_derived() { return *static_cast<Derived*>(this); }

 private:
  ValueType value_;
  DerivativesType derivatives_;
};

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake
