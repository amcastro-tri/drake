/** @file
A matrix and its derivatives. */

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
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"
// #include "drake/common/symbolic/expression.h"
// #include "drake/math/autodiff_gradient.h"

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
  static constexpr bool sparse_derivatives =
      Traits<DerivedType>::sparse_derivatives;
  typedef typename std::conditional<
      sparse_derivatives, std::vector<std::optional<PartialsType>>,
      std::vector<PartialsType>>::type DerivativesType;

  /** Constructs object with uninitialized value and no derivatives. */
  ObjectWithDerivatives() {}

  ObjectWithDerivatives(ValueType value) : value_(std::move(value)) {}

  ObjectWithDerivatives(ValueType value, DerivativesType derivatives)
      : value_(std::move(value)), derivatives_(std::move(derivatives)) {}

  // Reset both value and derivatives.
  void Reset(ValueType value, DerivativesType derivatives) {
    value_ = std::move(value);
    derivatives_ = std::move(derivatives);
  }

  int num_derivatives() const { return derivatives_.size(); }

  const ValueType& value() const { return value_; }
  const DerivativesType& derivatives() const { return derivatives_; }

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
  DerivativesType derivatives_;
};

template <class Operation>
class ObjectWithDerivativesBinaryOperation {
 public:
  using LhsType = Operation::LhsType;
  using RhsType = Operation::RhsType;
  using ResultType = Operation::ResultType;
  static constexpr bool sparse_derivatives =
      LhsType::sparse_derivatives && RhsType::sparse_derivatives;

  // They must both be either dense or sparse.
  static_assert(LhsType::sparse_derivatives == RhsType::sparse_derivatives);

  // TODO: Static cast that LhsType, RhsType, and ResultType are
  // ObjectWithDerivatives.

  // TODO: Implement sparse version.
  static ResultType Calc(const LhsType& lhs, const RhsType& rhs) {
    if constexpr (sparse_derivatives) {
      return CalcSparse(lhs, rhs);
    }
    else {
      return CalcDense(lhs, rhs);
    }
  }

 private:  
  static ResultType CalcDense(const LhsType& lhs, const RhsType& rhs) {
    typename ResultType::ValueType value =
        Operation::CalcValue(lhs.value(), rhs.value());
    DRAKE_DEMAND(lhs.num_derivatives() == rhs.num_derivatives());
    std::vector<typename ResultType::PartialsType> derivatives(
        lhs.num_derivatives());
    for (int i = 0; i < lhs.num_derivatives(); ++i) {
      const typename LhsType::PartialsType& lhs_partial = lhs.derivatives()[i];
      const typename RhsType::PartialsType& rhs_partial = rhs.derivatives()[i];
      derivatives[i] = Operation::CalcPartial(value, lhs.value(), lhs_partial,
                                              rhs.value(), rhs_partial);
    }
    return ResultType(std::move(value), std::move(derivatives));
  }

  static ResultType CalcSparse(const LhsType& lhs, const RhsType& rhs) {
    typename ResultType::ValueType value =
        Operation::CalcValue(lhs.value(), rhs.value());
    DRAKE_DEMAND(lhs.num_derivatives() == rhs.num_derivatives());

    

    std::vector<typename ResultType::PartialsType> derivatives(
        lhs.num_derivatives());
    for (int i = 0; i < lhs.num_derivatives(); ++i) {
      const typename LhsType::PartialsType& lhs_partial = lhs.derivatives()[i];
      const typename RhsType::PartialsType& rhs_partial = rhs.derivatives()[i];
      derivatives[i] = Operation::CalcPartial(value, lhs.value(), lhs_partial,
                                              rhs.value(), rhs_partial);
    }
    return ResultType(std::move(value), std::move(derivatives));
  }
};


}  // namespace internal
}  // namespace math
}  // namespace drake
