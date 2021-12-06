#pragma once

#include <iostream>
#include <memory>
#include <variant>
#include <vector>

#include "drake/common/eigen_types.h"
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// TODO: make this work.
// Represents a permutation p such that u[p(i)] = x[i].
class PartialPermutation {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PartialPermutation);

  explicit PartialPermutation(std::vector<int>&& permutation);

  int domain_size() const { return permutation_.size(); }
  int permuted_domain_size() const { return inverse_permutation_.size(); }

  // TODO: templatize on class type, rather than scalar type. The only
  // reuirement being to have [] operator (or do it in temrs of iterators?)
  // N.B. requirement could be to suppor range iteration, when we start using
  // Eigen 3.4
  // PointerToPermutedVectorType must resolve to a pointer to a class of type
  // PermutedVectorType. Both VectorType and PermutedVectorType must implement:
  //  - VectorType::size().
  //  - VectorType::operator[](int).  
  template <class VectorType, class PointerToPermutedVectorType>
  void Apply(const VectorType& x, PointerToPermutedVectorType x_permuted) const {
    DRAKE_DEMAND(static_cast<int>(x.size()) == domain_size());
    DRAKE_DEMAND(x_permuted != nullptr);
    DRAKE_DEMAND(static_cast<int>(x_permuted->size()) ==
                 permuted_domain_size());
    for (int i_permuted = 0; i_permuted < permuted_domain_size();
         ++i_permuted) {
      const int i = inverse_permutation_[i_permuted];
      (*x_permuted)[i_permuted] = x[i];
    }
  }

  template <class PermutedVectorType, class PointerToVectorType>
  void ApplyInverse(const PermutedVectorType& x_permuted,
                    PointerToVectorType x) const {
    DRAKE_DEMAND(static_cast<int>(x_permuted.size()) == permuted_domain_size());
    DRAKE_DEMAND(x != nullptr);
    DRAKE_DEMAND(static_cast<int>(x->size()) == domain_size());        
    for (int i_permuted = 0; i_permuted < permuted_domain_size();
         ++i_permuted) {
      const int i = inverse_permutation_[i_permuted];
      (*x)[i] = x_permuted[i_permuted];
    }
  }

 private:
  std::vector<int> permutation_;
  std::vector<int> inverse_permutation_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
