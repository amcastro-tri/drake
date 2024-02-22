/** @file
A matrix and its derivatives. */

#pragma once

#include <algorithm>
#include <cmath>
#include <ostream>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

//#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"
//#include "drake/common/symbolic/expression.h"
//#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace math {
namespace internal {

template <class DerivedObjectWithDerivatives>
struct Traits {};

template <class DerivedType>
class ObjectWithDerivatives {  
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ObjectWithDerivatives);

  using ValueType = Traits<DerivedType>::ValueType;
  using PartialsType = Traits<DerivedType>::PartialsType;

  /** Constructs object with uninitialized value and no derivatives. */
  ObjectWithDerivatives() {}

  ObjectWithDerivatives(ValueType value) : value_(std::move(value)) {}

  ObjectWithDerivatives(ValueType value, std::vector<PartialsType> derivatives)
      : value_(std::move(value)), derivatives_(std::move(derivatives)) {}

  int num_derivatives() const { return derivatives_.size(); }

  const ValueType& value() const { return value_; }
  const std::vector<PartialsType>& derivatives() const { return derivatives_; }

  const DerivedType& get_derived() const {
    // Static cast is safe since types are resolved at compile time by CRTP.
    return *static_cast<const DerivedType*>(this);
  }  

 private:
  DerivedType& get_mutable_derived() {
    // Static cast is safe since types are resolved at compile time by CRTP.
    return *static_cast<DerivedType*>(this);
  }

  ValueType value_;
  // TODO: Implement sparse version.
  std::vector<PartialsType> derivatives_;
};

template <class Operation>
class ObjectWithDerivativesBinaryOperation {
  public:
    using LhsType = Operation::LhsType;
    using RhsType = Operation::RhsType;
    using ResultType = Operation::ResultType;

    // TODO: Static cast that LhsType, RhsType, and ResultType are
    // ObjectWithDerivatives.

    // TODO: Implement sparse version.
    static ResultType Calc(const LhsType& lhs, const RhsType& rhs) {
      typename ResultType::ValueType value =
          Operation::CalcValue(lhs.value(), rhs.value());
      DRAKE_DEMAND(lhs.num_derivatives() == rhs.num_derivatives());
      std::vector<typename ResultType::PartialsType> derivatives(
          lhs.num_derivatives());
      for (int i = 0; i < lhs.num_derivatives(); ++i) {
        const typename LhsType::PartialsType& lhs_partial =
            lhs.derivatives()[i];
        const typename RhsType::PartialsType& rhs_partial =
            rhs.derivatives()[i];
        derivatives[i] = Operation::CalcPartial(value, lhs.value(), lhs_partial,
                                                rhs.value(), rhs_partial);
      }
      return ResultType(std::move(value), std::move(derivatives));
    }
};


#if 0
template <class ValueType, class PartialsType>
class DiffObject {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiffObject);

  /** Constructs with an uninitialized matrix and no derivatives. */
  DiffObject() {}

  /** Returns the stored value with scalars of type double. */
  const ValueType& value() const { return value_; }

  /** Returns the kᵗʰ non-zero derivative. The corresponding variable is
  vᵢ, where i = non_zero(k). That is, we're returning ∂M/∂vᵢ, where M
  is the mathematical object returned by value(). */
  const PartialsType& non_zero_derivative(int k) const {
    DRAKE_ASSERT(0 <= k && k < static_cast<int>(non_zero_derivatives_.size()));
    return non_zero_derivatives_[k];
  }

  /** Returns the variable number i for the kᵗʰ non-zero derivative. That is,
  the kᵗʰ non-zero partial derivative of this object M is ∂M/∂vi. */
  int non_zero(int k) const {
    DRAKE_ASSERT(0 <= k && k < num_non_zeros());
    return non_zeros_[k];
  }

  /** Returns the number of partial derivatives whose values are stored
  explicitly, rather than being implicitly considered zero. */
  int num_non_zeros() const { return static_cast<int>(non_zeros_.size()); }

  /** Returns the total number of variables with respect to which we are taking
  derivatives, including those explicitly stored and implicitly zero. If the
  number of variables is returned zero, that means we don't know how may
  variables there are but regardless of that, all the derivatives are known to
  be zero. That representation is best for a known-constant matrix. */
  int num_variables() const { return num_variables_; }

  template <class RhsValueType, class RhsPartialsType, class OpResultType,
            class OpPartialsType>
  auto ApplyBinaryOperation(
      const DiffObject<RhsValueType, RhsPartialsType>& rhs,
      std::function<OpResultType(const LhsValueType&, const RhsValueType&)> op,
      std::function<OpPartialsType(const OpResultType& const LhsValueType&,
                                   const LhsPartialsType&, const RhsValueType&,
                                   const RhsPartialsType&)>
          op_derivs,
      std::function<OpPartialsType(const OpResultType& const LhsValueType&,
                                   const LhsPartialsType&, const RhsValueType&,
                                   const RhsPartialsType&)>
          op_derivs_zero_lhs,
      std::function<OpPartialsType(const OpResultType& const LhsValueType&,
                                   const LhsPartialsType&, const RhsValueType&,
                                   const RhsPartialsType&)>
          op_derivs_zero_rhs) const -> DiffObject<OpResultType, OpPartialsType>;

 private:
};
#endif

}  // namespace internal
}  // namespace math
}  // namespace drake
