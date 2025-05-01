#pragma once

#include <numeric>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/fast_sap/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

template <typename T>
class PatchConstraintDataPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PatchConstraintDataPool);

  // The number of patches in the pool.
  int num_patches() const { return num_patches_; }

  // The total number of contact pairs among all patches.
  int num_pairs() const { return num_pairs_; }

  /* Default constructor for an empty pool. */
  PatchConstraintDataPool() = default;

  PatchConstraintDataPool(const std::vector<int>& patch_size,
                          const std::vector<int>& num_velocities) {
    Resize(patch_size, num_velocities);
  }

  /* @param patch_size Number of contact pairs for the k-th patch.
     @param num_velocities Number of velocities for the k-th patch. */
  void Resize(const std::vector<int>& patch_size,
              const std::vector<int>& num_velocities) {
    DRAKE_ASSERT(patch_size.size() == num_velocities.size());
    num_patches_ = ssize(patch_size);
    num_pairs_ = std::accumulate(patch_size.begin(), patch_size.end(), 0);
    vc_.Resize(num_pairs_);
    gamma_.Resize(num_pairs_);
    gradient_.Resize(num_velocities);
    H_.Resize(num_velocities, num_velocities);
  }

  const Vector3<T>& vc(int pair_index) const {
    DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs());
    return vc_[pair_index];
  }
  Vector3<T>& vc(int pair_index) {
    DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs());
    return vc_[pair_index];
  }
  const Vector3<T>& gamma(int pair_index) const {
    DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs());
    return gamma_[pair_index];
  }
  Vector3<T>& gamma(int pair_index) {
    DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs());
    return gamma_[pair_index];
  }

 private:
  int num_patches_{0};
  int num_pairs_{0};

  T cost;

  // Per-pair quantities, of size num_pairs().
  EigenPool<Vector3<T>> vc_;     // contact velocity.
  EigenPool<Vector3<T>> gamma_;  // Per contact pair impulse.

  // Per-patch quantities, of size num_patches():
  EigenPool<VectorX<T>> gradient_;  // ∇ℓₖ = -Jₖᵀ⋅γₖ
  EigenPool<MatrixX<T>> H_;         // Hₖ = Jₖᵀ⋅Gₖ⋅Jₖ
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
