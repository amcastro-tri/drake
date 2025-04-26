

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

// Forward declaration.
template <typename T>
class PatchConstraintParamsView;

// A bunch of std::vector with the actual memory storage for all params in a
// SapModel
template <typename T>
class PatchConstraintParamsPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PatchConstraintParamsPool);

  using Vector3View = EigenPool<Vector3<T>>::ElementView;
  using Matrix3View = EigenPool<Matrix3<T>>::ElementView;
  using JacobianView = Eigen::Map<Matrix6X<T>>;

  int num_constraints() { return num_patches_; }

  int num_constraint_equations() { return 3 * num_constraints(); }

  /* Resizes to store patch constraint data. No memory allocation performed if
     the current capcity is enough to store this data size.
   @param num_patches The number of patches.
   @param num_pairs_capcity Capacity for the total number of pairs.
   @param max_clique_size Used to estimate storage for spatial velocity
     Jacobians.  */
  void Resize(int num_patches, int num_pairs_capacity, int max_clique_size) {
    // per-patch data.
    num_pairs_.resize(num_patches);
    JA_first_col_.resize(num_patches);
    JB_first_col_.resize(num_patches);
    J_WA_cols_.Resize(num_patches * max_clique_size);
    J_WB_cols_.Resize(num_patches * max_clique_size);
    dissipation_.resize(num_patches);
    friction_.resize(num_patches);

    // per-pair data.
    p_AoC_W_.Resize(num_pairs_capacity);
    p_BoC_W_.Resize(num_pairs_capacity);
    R_WC_.Resize(num_pairs_capacity);
    stiffness_.resize(num_pairs_capacity);
    fn0_.resize(num_pairs_capacity);
  }

  int num_patches() const { return ssize(num_pairs); }

  int num_pairs(int patch_index) const {
    DRAKE_ASSERT(0 <= patch_index && patch_index < num_patches());
    return num_pairs_[patch_index];
  }

 private:
  // Pools of size num_patches().
  // std::vector<double> stiction_tolerance_;  // Stiction tolerance.
  // std::vector<double> sap_sigma_;  // SAP's regularization parameter.
  std::vector<int> num_pairs_;   // Number of pairs per patch.  
  std::vector<int> JA_first_col_;
  EigenPool<Vector6<T>> J_WA_cols_;  // Spatial velocity Jacobian columns for A.
  std::vector<int> JB_first_col_;
  EigenPool<Vector6<T>> J_WB_cols_;  // Spatial velocity Jacobian columns for B.
  std::vector<T> dissipation_;   // Hunt & Crossley dissipation.
  std::vector<T> friction_;      // Friction coefficient.

  // Pools of size total_num_pairs_.
  EigenPool<Vector3<T>> p_AoC_W_;  // Contact point position.
  EigenPool<Vector3<T>> p_BoC_W_;  // Contact point position.
  EigenPool<Matrix3<T>> R_WC_;     // Contact frame orientation.
  std::vector<T> stiffness_;       // Linear stiffness, N/m.
  std::vector<T> fn0_;             // Previous time step normal force.
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