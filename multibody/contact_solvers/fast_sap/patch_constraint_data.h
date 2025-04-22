

#pragma once

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/fast_sap/eigen_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

/* Notation

 Indexes:
   k: constraint
   i: clique
   p: contact pair
 Sizes:
   n: num velocities. Thus:
      - nᵢ is the number of velocities for the i-th clique.
      - nₖ is the total number of velocities for the k-th constraint
   m: num constraint equations. i.e. mₖ applies to k-th constraint.
*/

// A bunch of std::vector with the actual memory storage for all params in a
// SapModel
template <typename T>
class PatchConstraintParamsPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PatchConstraintParamsPool);

 private:
  // Sizes.
  int num_patches_;                     // Number of patch constraints.
  int total_num_pairs_;                 // Total number of pairs.
  std::vector<int> num_contact_pairs_;  // Of size num_patches_.

  // Pools of size total_num_pairs_.
  EigenPool<Vector3<T>> p_WC_;  // Contact point position.
  EigenPool<Matrix3<T>> R_WC_;  // Contact frame orientation.
  std::vector<T> stiffness_;    // Linear stiffness, N/m.
  std::vector<T> fn0_;          // Previous time step normal force.

  // Pools of size num_patches_.
  EigenPool<Matrix6X<T>> J_WA_;  // Spatial velocity Jaocobians for A.
  EigenPool<Matrix6X<T>> J_WB_;  // Spatial velocity Jaocobians for B.
  std::vector<T> dissipation_;   // Hunt & Crossley dissipation.
  std::vector<T> friction_;      // Friction coefficient.
};

template <typename T>
class PatchConstraintDataPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PatchConstraintDataPool);

  int num_patches() const { return num_patches_; }

  /* Default constructor for an empty pool. */
  PatchConstraintDataPool() = default;

  /* @param num_equations Number of contact pairs for the k-th patch.
     @param num_velocities Number of velocities for the k-th patch. */
  void Resize(const std::vector<int>& patch_size,
              const std::vector<int>& num_velocities)
      : num_patches_(ssize(patch_size)) {
    DRAKE_ASSERT(patch_size.size() == num_velocities.size());
    vc_.Resize(patch_size);
    gamma_.Resize(patch_size);
    gradient_.Resize(num_velocities);
    H_.Resize(num_velocities, num_velocities);
  }

 private:
  int num_patches_{0};

  T cost;

  // Per-pair quantities, of size num_pairs().
  EigenPool<Vector3<T>> vc_;     // contact velocity.
  EigenPool<Vector3<T>> gamma_;  // Per contact pair impulse.

  // ∇ℓₖ = -Jₖᵀ⋅γₖ, of size num_patches().
  EigenPool<VectorX<T>> gradient_;

  // Patch Hessian matrix, Hₖ = Jₖᵀ⋅Gₖ⋅Jₖ, of size num_patches().
  EigenPool<MatrixX<T>> H_;
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake