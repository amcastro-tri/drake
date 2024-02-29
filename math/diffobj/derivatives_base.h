#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <ostream>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/unused.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace math {
namespace diffobj {
namespace internal {

// TODO: move this somewhere more appropriate.
template <class TypeWithTraits>
struct Traits {};

// Given the type Foo<Bar>, this method is used to retrieve the templated class
// Foo like so:
//   template <typename T>
//   using Template = get_template<SomeType>::template type<T>;
// So that Template<T> resolves to Foo<T>.
template <typename T>
struct get_template;

template <template <class...> class Y, typename... Args>
struct get_template<Y<Args...>> {
  template <typename... Others>
  using type = Y<Others...>;
};

template <template <class> class DerivativesDerived, class _PartialsType>
class DerivativesBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DerivativesBase);

  using PartialsType = _PartialsType;
  //using DerivativesType = Traits<Derived>::DerivativesType;
  //template <class PT>
  //using ContainerOfPartials = get_template<Derived>;
  template <class P>
  using ContainerOfPartials = DerivativesDerived<P>;
  using Derived = DerivativesDerived<PartialsType>;

  DerivativesBase() = default;

  // TODO: Such a constructor would require an additional class layer, where the
  // next layer does already know the memory layout to construct and object.
  // VectorOfPartialsBase is abstract, with not data members.
  // The hierarchy would look VectorOfPartialsBase <- VectorOfPartials <-
  // DenseVectorOfPartials (e.g.).
  // Constructs a VectorOfPartials from a
  // std::vector of non-zero partials. The newly constructed object will have
  // partials.size() number of variables. That is, this constructor makes a
  // "dense" vector of partials. explicit  
  // DerivativesBase(std::vector<PartialsType> partials);
  //
  // TODO: ditto for this method.
  // std::vector<Matrix3<double>> MakeDenseStdVectorOfPartials() const;

  const Derived& derived() const { return *static_cast<const Derived*>(this); }
  Derived& mutable_derived() { return *static_cast<Derived*>(this); }

  int num_variables() const { return derived().num_variables(); }

  // Sets the i-th partial derivative.
  // Refer to derived classes for further semantics.
  void SetPartial(int i, PartialsType partial) {
    mutable_derived().SetPartial(i, std::move(partial));
  }

  // Returns `true` iff unary_predicate is `true` for all the in-memory
  // partials.
  bool AllOf(std::function<bool(const PartialsType&)> unary_predicate) const {
    return derived().AllOf(unary_predicate);
  }

  // Returns `true` iff binary_predicate(lhs_i, rhs_i) is `true` for all pairs
  // of partials w.r.t. the i-th variable.
  // @pre sizes must match.
  // Returns `true` for zero in-memory derivatives.
  bool AllOf(const Derived& rhs,
             std::function<bool(const PartialsType&, const PartialsType&)>
                 binary_predicate) const {
    return derived().AllOf(rhs, binary_predicate);
  }

  // TODO: Make this take a functor as ApplyBinaryOperation() does. There is no
  // reason for an Opertaion class.
  template <class Operation>
  DerivativesDerived<typename Operation::ResultPartialsType>
  ApplyUnaryOperation() const {
    return derived().template ApplyUnaryOperation<Operation>();
  }

  template <class RhsDerivativesType, class ResultPartialsType>
  DerivativesDerived<ResultPartialsType> ApplyBinaryOperation(
      const RhsDerivativesType& rhs,
      std::function<
          ResultPartialsType(const PartialsType&,
                             const typename RhsDerivativesType::PartialsType&)>
          op) const {
    return derived().ApplyBinaryOperation(rhs, op);
  }
};

}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake
