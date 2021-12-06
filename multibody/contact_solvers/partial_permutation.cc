#include "drake/multibody/contact_solvers/partial_permutation.h"

#include <algorithm>

#include "fmt/format.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

PartialPermutation::PartialPermutation(std::vector<int>&& permutation)
    : permutation_(std::move(permutation)) {
  const int from_size = permutation_.size();      
  // Determine size of the permuted domain.
  const int to_size =
      *std::max_element(permutation_.begin(), permutation_.end()) + 1;

  if (to_size > from_size) {
    throw std::runtime_error(fmt::format(
        "The size of the permuted domain must be smaller or equal than that of "
        "the original domian. Index {}, larger or equal than the domain size, "
        "appears in the input permutation.",
        to_size - 1));
  }

  // Allocate inverse permutation and indicated values that are not present with
  // an invalid (-1) index.
  inverse_permutation_.resize(to_size, -1);  
  // Fill in inverse permutation and check for valid permuted entries.
  for (int i = 0; i < from_size; ++i) {
    const int i_permuted = permutation_[i];
    // Ignore negative entries since those correspond to indexes that do not
    // participate in the partial permutation.
    if (i_permuted >= 0) {
      if (inverse_permutation_[i_permuted] < 0) {
        inverse_permutation_[i_permuted] = i;
      } else {
        throw std::runtime_error(
            fmt::format("Index {} appears at least twice in the input "
                        "permutation. At {} and at {}.",
                        i_permuted, inverse_permutation_[i_permuted], i));
      }
    }
  }

  // Verify that all indexes in [0; to_size-1] were specified. This corresponds
  // to all indexes in inverse_permutation_ being positive.
  for (int i_permuted = 0; i_permuted < to_size; ++i_permuted) {
    if (inverse_permutation_[i_permuted] < 0) {
      throw std::runtime_error(
          fmt::format("Index {} not present in the permutation. However the "
                      "maximum specified permuted index is {}.",
                      i_permuted, to_size - 1));
    }
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
