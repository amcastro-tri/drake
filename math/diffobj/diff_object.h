#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <ostream>
#include <vector>

#include "math/diffobj/derivatives_base.h"

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff_gradient.h"

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
  using ValueType = typename Traits<Derived>::ValueType;
  using PartialsType = typename Traits<Derived>::PartialsType;
  template <class P>
  using DerivativesContainerType =
      typename Traits<Derived>::template DerivativesContainerType<P>;
  using DerivativesType = DerivativesContainerType<PartialsType>;

  DiffObject() = default;

  // Constructs a constant object, all derivatives are zero.
  DiffObject(ValueType value) : value_(std::move(value)) {}

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

  template <class Operation>
  typename Operation::ResultType ApplyBinaryOperation(
      const typename Operation::RhsType& rhs) const {
    DRAKE_DEMAND(num_variables() == rhs.num_variables());
    static_assert(std::is_same_v<Derived, typename Operation::LhsType>);
    //using RhsType = typename Operation::RhsType;
    //using RhsDerivativesContainerType =
    //    typename RhsType::DerivativesContainerType;

    // TODO: how to resolve static assert below?    
    //static_assert(
    //    std::is_same_v<DerivativesContainerType, RhsDerivativesContainerType>);

    using ResultType = typename Operation::ResultType;
    // TODO: also static assert ResultType's container type.

    const typename Operation::DataType data =
        Operation::CalcData(derived(), rhs);
    const typename Operation::ResultValueType result_value =
        Operation::CalcValue(data, value(), rhs.value());
    // TODO: Consider placing this lambda's captured data into a single struct
    // to minimize copying cost.
    std::function<typename Operation::ResultPartialsType(
        const typename Operation::LhsPartialsType*,
        const typename Operation::RhsPartialsType*)>
        partials_calc =
            [&data](const typename Operation::LhsPartialsType* lhs_partial,
                    const typename Operation::RhsPartialsType* rhs_partial) ->
        typename Operation::ResultPartialsType {
          return Operation::CalcPartial(data, lhs_partial, rhs_partial);
        };

    const typename ResultType::DerivativesType result_derivatives =
        derivatives().ApplyBinaryOperation(rhs.derivatives(), partials_calc);
    return typename Operation::ResultType(std::move(result_value),
                                          std::move(result_derivatives));
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
