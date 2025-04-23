

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

  using Vector3Pool = EigenPool<Vector3<T>>;
  using Vector3View = Vector3Pool::ElementView;
  using ConstVector3View = Vector3Pool::ConstElementView;
  using Matrix3Pool = EigenPool<Matrix3<T>>;
  using Matrix3View = Matrix3Pool::ElementView;
  using ConstMatrix3View = Matrix3Pool::ConstElementView;
  using MatrixXPool = EigenPool<MatrixX<T>>;
  using MatrixXView = MatrixXPool::ElementView;
  using ConstMatrixXView = MatrixXPool::ConstElementView;

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

  /* @param num_equations Number of contact pairs for the k-th patch.
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

  const ConstVector3View& vc(int pair_index) const {
    DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs());
    return vc_[pair_index];
  }
  Vector3View& vc(int pair_index) {
    DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs());
    return vc_[pair_index];
  }
  const ConstVector3View& gamma(int pair_index) const {
    DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs());
    return gamma_[pair_index];
  }
  Vector3View& gamma(int pair_index) {
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