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
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace math {
namespace diffobj {
namespace internal {

template <class PartialsType>
class OptionalDerivatives;  // Forward declaration for the Traits below.

// template <class _PartialsType>
// struct Traits<OptionalDerivatives<_PartialsType>> {
//   using PartialsType = _PartialsType;
// };

// Generic method to compare two partials.
// OptionalDerivatives::IsNearlyEqualTo() below relies on the existence of a
// specialization on PartialsType.
template <class PartialsType>
bool IsNearlyEqualTo(const PartialsType, const PartialsType);

template <class _PartialsType>
class OptionalDerivatives
    : public DerivativesBase<OptionalDerivatives, _PartialsType> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OptionalDerivatives);

  using PartialsType = _PartialsType;
  using Base = OptionalDerivatives<PartialsType>;

  OptionalDerivatives() = default;

  OptionalDerivatives(std::vector<std::optional<PartialsType>> derivatives)
      : derivatives_(derivatives) {}

  // Expensive constructor to make dense derivatives.
  // num_variables() will equal derivatives.size().  
  OptionalDerivatives(std::vector<PartialsType> derivatives) {
    derivatives_.resize(derivatives.size());
    for (int i = 0; i < ssize(derivatives_); ++i) {
      derivatives_[i] = std::move(derivatives[i]);
    }
  }

  // @pre We can call PartialsType::setZero().
  std::vector<PartialsType> MakeDenseStdVectorOfPartials() const {
    std::vector<PartialsType> derivatives(num_variables());
    for (int i = 0; i < num_variables(); ++i) {
      if (derivatives_[i].has_value()) {
        derivatives[i] = derivatives_[i].value();
      } else {
        derivatives[i].setZero();
      }
    }
    return derivatives;
  }

  int num_variables() const { return ssize(derivatives_); }

  bool IsExactlyZero() const {
    if (num_variables() == 0) return true;
    for (int i = 0; i < num_variables(); ++i) {
      if (derivatives_[i].has_value()) {
        if (!derivatives_[i]->isZero(0.0)) return false;
      }
    }
    return true;
  }

  bool AllOf(std::function<bool(const PartialsType&)> unary_predicate) const {
    for (int i = 0; i < num_variables(); ++i) {
      if (derivatives_[i].has_value() &&
          !unary_predicate(derivatives_[i].value()))
        return false;
    }
    return true;
  }

  bool AllOf(const OptionalDerivatives& rhs,
             std::function<bool(const PartialsType&, const PartialsType&)>
                 binary_predicate) const {
    DRAKE_DEMAND(num_variables() == rhs.num_variables());                  
    for (int i = 0; i < num_variables(); ++i) {
      if (derivatives_[i].has_value() != rhs.derivatives_[i].has_value())
        return false;

      if (derivatives_[i].has_value() &&
          !binary_predicate(*derivatives_[i], *rhs.derivatives_[i]))
        return false;
    }
    return true;
  }

#if 0
  // N.B. This method assumes the specialization of IsNearlyEqualTo(lhs, rhs)
  // where lhs and rhs are of type PartialsType.
  bool IsNearlyEqualTo(const OptionalDerivatives& other, double tolerance) {
    for (int i = 0; i < num_variables(); ++i) {
      if (derivatives_[i].has_value() != other.derivatives_[i].has_value())
        return false;
      if (derivatives_[i].has_value()) {
        if (!IsNearlyEqualTo(*derivatives_[i], *other.derivatives_[i],
                             tolerance))
          return false;
      }
    }
    return true;
  }
#endif  

  template <class Operation>
  OptionalDerivatives<typename Operation::ResultPartialsType>
  ApplyUnaryOperation() const {
    std::vector<std::optional<typename Operation::ResultPartialsType>> result(
        num_variables());
    for (int i = 0; i < num_variables(); ++i) {
      if (derivatives_[i].has_value()) {
        result[i] = Operation::Calc(derivatives_[i].value());
      }
    }
    return OptionalDerivatives<typename Operation::ResultPartialsType>(
        std::move(result));
  }

  template <class RhsDerivativesType, class ResultPartialsType>
  OptionalDerivatives<ResultPartialsType> ApplyBinaryOperation(
      const RhsDerivativesType& rhs,
      std::function<
          ResultPartialsType(const PartialsType*,
                             const typename RhsDerivativesType::PartialsType*)>
          op) const {
    DRAKE_DEMAND(num_variables() == rhs.num_variables());
    std::vector<std::optional<ResultPartialsType>> result(num_variables());
    for (int i = 0; i < num_variables(); ++i) {
      if (derivatives_[i].has_value() && rhs.derivatives_[i].has_value()) {
        result[i] = op(&derivatives_[i].value(), &rhs.derivatives_[i].value());
      } else if (derivatives_[i].has_value()) {
        result[i] = op(&derivatives_[i].value(), nullptr);
      } else {
        result[i] = op(nullptr, &rhs.derivatives_[i].value());
      }
    }
    return OptionalDerivatives<ResultPartialsType>(std::move(result));
  }

 private:
  std::vector<std::optional<PartialsType>> derivatives_;
};

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake
