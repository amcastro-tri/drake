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

  bool IsNearlyEqualTo(const Derived& other, double tolerance) {
    return derived().IsNearlyEqualTo(other, tolerance);
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
