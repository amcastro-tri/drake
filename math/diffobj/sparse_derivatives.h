#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <optional>
#include <ostream>
#include <vector>

#include "math/diffobj/derivatives_base.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"

namespace drake {
namespace math {
namespace diffobj {
namespace internal {

template <class PartialsType>
class SparseDerivatives;  // Forward declaration for the Traits below.

// template <class _PartialsType>
// struct Traits<SparseDerivatives<_PartialsType>> {
//   using PartialsType = _PartialsType;
// };

template <class _PartialsType>
class SparseDerivatives
    : public DerivativesBase<SparseDerivatives, _PartialsType> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SparseDerivatives);

  using PartialsType = _PartialsType;
  using Base = SparseDerivatives<PartialsType>;

  SparseDerivatives() = default;

  // Creates a SparseDerivatives of size with nv variables. Derivatives are
  // left un-initialized.
  explicit SparseDerivatives(int nv) : num_variables_(nv) {
    non_zeros_.reserve(nv);
    non_zero_partials_.reserve(nv);
  }

  SparseDerivatives(int num_variables, std::vector<int> non_zeros,
                    std::vector<PartialsType> non_zero_partials)
      : num_variables_(num_variables),
        non_zeros_(std::move(non_zeros)),
        non_zero_partials_(std::move(non_zero_partials)) {}

  // Expensive constructor to make dense derivatives.
  // num_variables() will equal derivatives.size().
  SparseDerivatives(std::vector<PartialsType> all_partials) {
    num_variables_ = all_partials.size();
    non_zeros_.resize(num_variables_);
    std::iota(non_zeros_.begin(), non_zeros_.end(), 0);
    non_zero_partials_ = std::move(all_partials);
  }

  // @pre We can call PartialsType::setZero().
  std::vector<PartialsType> MakeDenseStdVectorOfPartials() const {
    std::vector<PartialsType> derivatives(num_variables());
    std::for_each(derivatives.begin(), derivatives.end(), [](PartialsType& p) {
      p.setZero();
    });
    for (int k = 0; k < num_non_zeros(); ++k) {
      const int i = non_zeros_[k];
      derivatives[i] = non_zero_partials_[k];
    }
    return derivatives;
  }

  int num_variables() const { return num_variables_; }

  int num_non_zeros() const { return ssize(non_zeros_); }

  const std::vector<int>& non_zeros() const { return non_zeros_; }
  const std::vector<PartialsType>& non_zero_partials() const {
    return non_zero_partials_;
  }
  int non_zero(int k) const { return non_zeros_[k]; }
  const PartialsType& non_zero_partial(int k) const {
    return non_zero_partials_[k];
  }

  // Additional semantics: Only a next partial can be set, i.e. last time this
  // method was called with index j < i.
  void SetPartial(int i, PartialsType partial) {
    DRAKE_ASSERT(0 <= i && i < num_variables());
    DRAKE_DEMAND(i > non_zeros_.back());
    non_zeros_.push_back(i);
    non_zero_partials_.push_back(std::move(partial));
  }

  bool AllOf(std::function<bool(const PartialsType&)> unary_predicate) const {
    for (const PartialsType& p : non_zero_partials_) {
      if (!unary_predicate(p)) return false;
    }
    return true;
  }

  // Returns `true` iff this and rhs have the same number of non-zero partials
  // and this->partial(k) equals rhs.partial(k) for all k = 0...num_non_zeros().
  // This method returns false if the sparsity layout is different.
  bool AllOf(const SparseDerivatives& rhs,
             std::function<bool(const PartialsType&, const PartialsType&)>
                 binary_predicate) const {
    DRAKE_DEMAND(num_variables() == rhs.num_variables());
    if (num_non_zeros() != rhs.num_non_zeros()) return false;
    if (non_zeros_ != rhs.non_zeros_) return false;
    for (int k = 0; k < num_non_zeros(); ++k) {
      if (!binary_predicate(non_zero_partials_[k], rhs.non_zero_partials_[k]))
        return false;
    }
    return true;
  }

  template <class Operation>
  SparseDerivatives<typename Operation::ResultPartialsType>
  ApplyUnaryOperation() const {
    std::vector<PartialsType> result(num_non_zeros());
    for (int k = 0; k < num_non_zeros(); ++k) {
      result[k] = Operation::Calc(non_zero_partials_[k]);
    }
    return SparseDerivatives<typename Operation::ResultPartialsType>(
        num_variables_, non_zeros_, std::move(result));
  }

  template <class RhsDerivativesType, class ResultPartialsType>
  SparseDerivatives<ResultPartialsType> ApplyBinaryOperation(
      const RhsDerivativesType& rhs,
      std::function<
          ResultPartialsType(const PartialsType*,
                             const typename RhsDerivativesType::PartialsType*)>
          op) const {
    DRAKE_DEMAND(num_variables() == rhs.num_variables());

    // The number of variables in both operands must match or be zero.
    const int result_num_variables = [&]() {
      if (num_variables() == 0) {
        DRAKE_ASSERT(num_non_zeros() == 0);
        return rhs.num_variables();
      }
      if (rhs.num_variables() == 0) {
        DRAKE_ASSERT(rhs.num_non_zeros() == 0);
        return num_variables();
      }
      DRAKE_DEMAND(num_variables() == rhs.num_variables());
      return num_variables();
    }();

    // Useful aliases.
    const SparseDerivatives& lhs = *this;
    using LhsPartialsType = PartialsType;
    using RhsPartialsType = typename RhsDerivativesType::PartialsType;

    // Note that this assumes the non_zero arrays are sorted and unique.
    // We also behave as though lhs and rhs *values* are non-zero, so we get
    // a "non-zero" derivative if either of the source derivatives is non-zero,
    // even though the actual result might still be zero.
    std::vector<int> result_non_zeros;
    // We'll need at least this much storage but might need more.
    result_non_zeros.reserve(
        std::max(lhs.num_non_zeros(), rhs.num_non_zeros()));
    std::set_union(non_zeros().begin(), non_zeros().end(),
                   rhs.non_zeros().begin(), rhs.non_zeros().end(),
                   std::back_inserter(result_non_zeros));

    const int result_num_non_zeros = result_non_zeros.size();
    std::vector<ResultPartialsType> result_non_zero_partials(
        result_num_non_zeros);    

    int i = 0, j = 0, r = 0;  // non_zero indices for lhs, rhs, result
    for (; i < lhs.num_non_zeros() && j < rhs.num_non_zeros(); ++r) {
      const int nzi = lhs.non_zero(i);
      const int nzj = rhs.non_zero(j);
      if (nzi == nzj) {
        // Both partials are non-zero.
        const LhsPartialsType& lhs_partial = lhs.non_zero_partial(i++);
        const RhsPartialsType& rhs_partial = rhs.non_zero_partial(j++);
        result_non_zero_partials[r] = op(&lhs_partial, &rhs_partial);
      } else if (nzi < nzj) {
        // Rhs partial is zero.
        const LhsPartialsType& lhs_partial = lhs.non_zero_partial(i++);
        result_non_zero_partials[r] = op(&lhs_partial, nullptr);
      } else {  // nzj < nzi
        // Lhs partial is zero.
        const RhsPartialsType& rhs_partial = rhs.non_zero_partial(j++);
        result_non_zero_partials[r] = op(nullptr, &rhs_partial);
      }
    }
    // At most one of these two loops will execute.
    for (; i < lhs.num_non_zeros(); ++i, ++r) {
      const LhsPartialsType& lhs_partial = lhs.non_zero_partial(i);
      result_non_zero_partials[r] = op(&lhs_partial, nullptr);
    }
    for (; j < rhs.num_non_zeros(); ++j, ++r) {
      const RhsPartialsType& rhs_partial = rhs.non_zero_partial(j);
      result_non_zero_partials[r] = op(nullptr, &rhs_partial);
    }

    return SparseDerivatives<ResultPartialsType>(
        result_num_variables, std::move(result_non_zeros),
        std::move(result_non_zero_partials));
  }

 private:
  int num_variables_{0};
  // These two vectors are the same size.
  std::vector<int> non_zeros_;  // Strictly increasing order.
  std::vector<PartialsType> non_zero_partials_;
};

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake
