/** @file
A matrix and its derivatives. */

#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
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
  using DerivativesType = std::vector<PartialsType>;

  /** Constructs object with uninitialized value and no derivatives. */
  ObjectWithDerivatives() {}

  ObjectWithDerivatives(ValueType value) : value_(std::move(value)) {}

  ObjectWithDerivatives(ValueType value, int num_variables,
                        int reserve_num_non_zeros)
      : value_(std::move(value)), num_variables_(num_variables) {
    non_zeros_.reserve(reserve_num_non_zeros);
    non_zero_derivatives_.reserve(reserve_num_non_zeros);
  }

  ObjectWithDerivatives(ValueType value, int num_variables,
                        std::vector<int> non_zeros,
                        std::vector<PartialsType> non_zero_derivatives)
      : value_(std::move(value)),
        num_variables_(num_variables),
        non_zeros_(std::move(non_zeros)),
        non_zero_derivatives_(std::move(non_zero_derivatives)) {}

  // This constructor assumes "dense" derivatives. That is, the number of
  // variables equal derivatives.size(). Therefore after this call,
  // num_non_zeros() equals num_derivatives().
  ObjectWithDerivatives(ValueType value, DerivativesType derivatives)
      : 
      value_(std::move(value)),
        non_zero_derivatives_(std::move(derivatives)) {
    num_variables_ =non_zero_derivatives_.size();
    non_zeros_.resize(num_variables_);
    std::iota(non_zeros_.begin(), non_zeros_.end(), 0);
  }

  // Reset both value and derivatives.
  // TODO: add num_variables arg.
  void Reset(ValueType value, DerivativesType derivatives) {
    value_ = std::move(value);
    non_zero_derivatives_ = std::move(derivatives);
  }

  void SetNextDerivative(int i, PartialsType ith_partial) {
    DRAKE_ASSERT(0 <= i && i < num_variables());
    DRAKE_DEMAND(non_zeros_.back() < i);
    non_zeros_.push_back(i);
    non_zero_derivatives_.push_back(std::move(ith_partial));
  }

  int num_variables() const { return num_variables_; }

  // TODO: remove. Use num_variables() instead.
  int num_derivatives() const { return num_variables(); }

  const ValueType& value() const { return value_; }

  int num_non_zeros() const { return static_cast<int>(non_zeros_.size()); }

  int non_zero(int k) const {
    DRAKE_ASSERT(0 <= k && k < num_non_zeros());
    return non_zeros_[k];
  }

  // TODO: consider "non_zero_partial()"
  const PartialsType& non_zero_derivative(int k) const {
    DRAKE_ASSERT(0 <= k && k < num_non_zeros());
    return non_zero_derivatives_[k];
  }

  const std::vector<int>& non_zeros() const { return non_zeros_; }

  // TODO: Rename to non_zero_derivatives().
  const DerivativesType& derivatives() const { return non_zero_derivatives_; }

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
  int num_variables_{0};
  // These two vectors are the same size.
  std::vector<int> non_zeros_;  // Strictly increasing order.
  DerivativesType non_zero_derivatives_;
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
    // The number of variables in both operands must match or be zero.
    const int num_variables = [&]() {
      if (lhs.num_variables() == 0) {
        DRAKE_ASSERT(lhs.num_non_zeros() == 0);
        return rhs.num_variables();
      }
      if (rhs.num_variables() == 0) {
        DRAKE_ASSERT(rhs.num_non_zeros() == 0);
        return lhs.num_variables();
      }
      DRAKE_DEMAND(lhs.num_variables() == rhs.num_variables());
      return lhs.num_variables();
    }();

    // Note that this assumes the non_zero arrays are sorted and unique.
    // We also behave as though lhs and rhs *values* are non-zero, so we get
    // a "non-zero" derivative if either of the source derivatives is non-zero,
    // even though the actual result might still be zero.
    std::vector<int> non_zeros;
    // We'll need at least this much storage but might need more.
    non_zeros.reserve(std::max(lhs.num_non_zeros(), rhs.num_non_zeros()));
    std::set_union(lhs.non_zeros().begin(), lhs.non_zeros().end(),
                   rhs.non_zeros().begin(), rhs.non_zeros().end(),
                   std::back_inserter(non_zeros));

    const int num_non_zeros = non_zeros.size();
    std::vector<typename ResultType::PartialsType> non_zero_derivatives(
        num_non_zeros);

    // Compute value.
    typename ResultType::ValueType value =
        Operation::CalcValue(lhs.value(), rhs.value());

    // Compute derivatives.
    // ∂a⋅b       ∂ b     ∂ a
    // ---- = a ⋅ ---  +  --- ⋅ b
    //  ∂vₖ       ∂vₖ     ∂vₖ

    int i = 0, j = 0, r = 0;  // non_zero indices for lhs, rhs, result
    for (; i < lhs.num_non_zeros() && j < rhs.num_non_zeros(); ++r) {
      const int nzi = lhs.non_zero(i);
      const int nzj = rhs.non_zero(j);
      if (nzi == nzj) {
        // Both partials are non-zero.
        const typename LhsType::PartialsType& lhs_partial =
            lhs.non_zero_derivative(i++);
        const typename RhsType::PartialsType& rhs_partial =
            rhs.non_zero_derivative(j++);
        non_zero_derivatives[r] = Operation::CalcPartial(
            value, lhs.value(), &lhs_partial, rhs.value(), &rhs_partial);
      } else if (nzi < nzj) {
        // Rhs partial is zero.
        const typename LhsType::PartialsType& lhs_partial =
            lhs.non_zero_derivative(i++);
        non_zero_derivatives[r] = Operation::CalcPartial(
            value, lhs.value(), &lhs_partial, rhs.value(), nullptr);
      } else {  // nzj < nzi
        // Lhs partial is zero.
        const typename LhsType::PartialsType& rhs_partial =
            rhs.non_zero_derivative(j++);
        non_zero_derivatives[r] = Operation::CalcPartial(
            value, lhs.value(), nullptr, rhs.value(), &rhs_partial);
      }
    }
    // At most one of these two loops will execute.
    for (; i < lhs.num_non_zeros(); ++i, ++r) {
      non_zero_derivatives[r] = Operation::CalcPartial(
          value, lhs.value(), &lhs.non_zero_derivative(i), rhs.value(),
          nullptr);
    }
    for (; j < rhs.num_non_zeros(); ++j, ++r) {
      non_zero_derivatives[r] =
          Operation::CalcPartial(value, lhs.value(), nullptr, rhs.value(),
                                 &rhs.non_zero_derivative(j));
    }

    return ResultType(std::move(value), num_variables, std::move(non_zeros),
                      std::move(non_zero_derivatives));
  }
};


}  // namespace internal
}  // namespace math
}  // namespace drake
