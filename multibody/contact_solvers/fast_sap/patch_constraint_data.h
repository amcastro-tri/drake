

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

  int num_constraints() { return num_patches_; }

  int num_constraint_equations() { return 3 * num_constraints(); }

  /* Adds parameters for a new patch constraint.
   @returns the index of the new patch constraint. */
  int Add(std::array<int, 2> cliques, std::array<int, 2> clique_nv,
          const std::vector<Vector3<T>>& p_WC,
          const std::vector<Matrix3<T>>& R_WC, std::vector<T> stiffness);

  /* Only if needed, dynamically allocates memory to fit the requested sizes.
   If allocation happens, data is lost.
   @returns true if dynamics memory allocation happened. */
  bool Resize(int num_patches, int num_pairs, int max_jac_cols);

  int num_patches() const { return num_patches_; }

  int num_pairs(int patch_index) const {
    DRAKE_ASSERT(0 <= patch_index && patch_index < num_patches());
    return num_pairs_[patch_index];
  }

 private:
  // Views can provided limited access to pool's internals.
  friend class PatchConstraintParamsView<T>;

  // Pools of size num_patches_.
  int num_patches_;              // Number of patch constraints.
  // std::vector<double> stiction_tolerance_;  // Stiction tolerance.
  // std::vector<double> sap_sigma_;  // SAP's regularization parameter.
  std::vector<int> num_pairs_;   // Number of pairs per patch.
  EigenPool<Matrix6X<T>> J_WA_;  // Spatial velocity Jacobians for A.
  EigenPool<Matrix6X<T>> J_WB_;  // Spatial velocity Jacobians for B.
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
class PatchConstraintParamsView {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PatchConstraintParamsView);

  /* View into a pool of patch constraint parameters.
   @param pool The patch constraints pool. Must remain valid for the lifetime of
   this view. */
  PatchConstraintParamsView(int patch_index, int first_pair_index,
                            PatchConstraintParamsPool* pool)
      : patch_index_(patch_index),
        first_pair_index_(first_pair_index),
      : pool_(*pool) {}

  /* Adds contact pair for this patch. There is no memory allocation if the pool
  already has capacity for  it. */
  void AddPair(const T& fn0, const T& stiffness, const Matrix3<T>& R_WC,
               const Vector3<T>& p_AoC_W, const Vector3<T>& p_BoC_W) {
    DRAKE_ASSERT(patch_index_ < ssize(pool.num_pairs_));
    // const int pair_index = pool_.num_pairs_[patch_index_]++;
    pool.fn0_.push_back(fn0);
    pool.stiffness_.push_back(stiffness);
    pool.R_WC.AddAndCopy(R_WC);
    pool.p_AoC_W.AddAndCopy(p_AoC_W);
    pool.p_BoC_W.AddAndCopy(p_BoC_W);
  }
  

  // Access patch data.
  const T& dissipation() const { return pool_.dissipation_[patch_index_]; }
  const T& friction() const { return pool_.friction_[patch_index_]; }
  const Matrix6X<T>& J_WA() const { return pool_.J_WA_[patch_index_]; }
  const Matrix6X<T>& J_WB() const { return pool_.J_WB_[patch_index_]; }

  // Access pair data.
  int num_pairs() const { return pool_.num_pairs(patch_index_); }
  const T& fn0(int pair_index) const { return pool_.fn0_[pair_index]; }
  const T& stiffness(int pair_index) const {
    return pool_.stiffness_[pair_index];
  }
  const Vector3<T>& p_AoC_W(int pair_index) const {
    DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs());
    return pool_.p_AoC_W_[pair_index];
  }
  const Vector3<T>& p_BoC_W(int pair_index) const {
    DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs());
    return pool_.p_BoC_W_[pair_index];
  }
  const Matrix3<T>& R_WC(int pair_index) const {
    DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs());
    return pool_.R_WC_[pair_index];
  }

 private:
  int patch_index_, first_pair_index_;
  PatchConstraintParamsPool<T>& pool_;
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