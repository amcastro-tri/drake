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
// PooledSapModel
template <typename T>
class PatchConstraintsPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PatchConstraintsPool);

  using JacobianView = Eigen::Map<Matrix6X<T>>;

  int num_constraints() { return ssize(num_pairs_); }

  int num_constraint_equations() { return 3 * total_num_pairs(); }

  /* Constructor for an empty pool. */
  PatchConstraintsPool() {
    JA_first_col_.push_back(0);
    JB_first_col_.push_back(0);
  }

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

    // The first column of the first pair is always at index = 0.
    JA_first_col_.push_back(0);
    JB_first_col_.push_back(0);
  }

  /* Reserve to store patch constraint data. No memory allocation performed if
     the current capcity is enough to store this data size.
   @param num_patches The number of patches.
   @param num_pairs_capcity Capacity for the total number of pairs.
   @param max_clique_size Used to estimate storage for spatial velocity
     Jacobians.  */
  void Reserve(int num_patches, int num_pairs_capacity, int max_clique_size) {
    // per-patch data.
    num_pairs_.reserve(num_patches);
    patch_num_velocities_.reserve(num_patches);
    JA_first_col_.reserve(num_patches);
    JB_first_col_.reserve(num_patches);
    J_WA_cols_.Reserve(num_patches * max_clique_size);
    J_WB_cols_.Reserve(num_patches * max_clique_size);
    dissipation_.reserve(num_patches);
    friction_.reserve(num_patches);

    // per-pair data.
    p_AoC_W_.Reserve(num_pairs_capacity);
    p_BoC_W_.Reserve(num_pairs_capacity);
    R_WC_.Reserve(num_pairs_capacity);
    stiffness_.reserve(num_pairs_capacity);
    fn0_.reserve(num_pairs_capacity);

    // The first column of the first pair is always at index = 0.
    JA_first_col_.push_back(0);
    JB_first_col_.push_back(0);
  }

  /* Pushes per-patch data for a new patch.  
   Per-pair data must be added with subsequent calls to PushBackPairData().
   @returns index to the new patch. */
  int PushBackPatchData(std::array<int, 2> cliques, const Vector6<T>& V_WA,
                        const Matrix6X<T>& J_WA, const Vector6<T>& V_WB,
                        const Matrix6X<T>& J_WB, const T& dissipation,
                        const T& friction, double vs, double sigma) {
    if (cliques[0] < 0) DRAKE_ASSERT(J_WA.cols() == 0);
    if (cliques[1] < 0) DRAKE_ASSERT(J_WB.cols() == 0);

    const int index = num_patches();

    num_pairs_.push_back(0);
    cliques_.push_back(cliques);
    const int Anv = J_WA.cols();
    const int Bnv = J_WB.cols();
    cliques_nv_.push_back({Anv, Bnv});    

    patch_num_velocities_.push_back(Anv + Bnv);

    V_WA_.PushBack(V_WA);
    V_WB_.PushBack(V_WB);

    // First column for the next pair.
    JA_first_col_.push_back(JA_first_col_.back() + J_WA.cols());
    JB_first_col_.push_back(JB_first_col_.back() + J_WB.cols());

    // Copy Jacobian columns.
    for (int i = 0; i < J_WA.cols(); ++i) {
      J_WA_cols_.PushBack(J_WA.col(i));
    }
    for (int i = 0; i < J_WB.cols(); ++i) {
      J_WB_cols_.PushBack(J_WB.col(i));
    }

    dissipation_.push_back(dissipation);
    friction_.push_back(friction);
    stiction_tolerance_.push_back(vs);
    sigma_.push_back(sigma);

    return index;
  }

  /* Adds per-pair data associated with the per-patch data last added with
  PushBackPatchData(), that is, the patch with index equal to num_patches() - 1. */
  void PushBackPairData(const T& fn0, const T& stiffness,
                        const Matrix3<T>& R_WC, const Vector3<T>& p_AoC_W,
                        const Vector3<T>& p_BoC_W) {
    const int patch_index = num_patches() - 1;
    ++num_pairs_[patch_index];

    fn0_.push_back(fn0);
    stiffness_.push_back(stiffness);
    R_WC_.PushBack(R_WC);
    p_AoC_W_.PushBack(p_AoC_W);
    p_BoC_W_.PushBack(p_BoC_W);
  }

  /* Clears memory, no memory is freed, and the capacity remains the same. 
   After this call num_patches() and num_pairs() equal zero. */
  void Clear() {
    // Clear per-patch pools.
    num_pairs_.clear();
    cliques_.clear();
    cliques_nv_.clear();
    patch_num_velocities_.clear();
    V_WA_.Clear();
    V_WB_.Clear();
    dissipation_.clear();
    friction_.clear();
    stiction_tolerance_.clear();
    sigma_.clear();

    // Jacobian pools.
    JA_first_col_.clear();
    JB_first_col_.clear();
    J_WA_cols_.Clear();
    J_WB_cols_.Clear();

    // Clear per-pair pools.    
    p_AoC_W_.Clear();
    p_BoC_W_.Clear();
    R_WC_.Clear();
    stiffness_.clear();
    fn0_.clear();

    // First column arrays must always have their first entry pointint to zero.
    JA_first_col_.push_back(0);
    JB_first_col_.push_back(0);
  }

  int num_patches() const { return ssize(num_pairs_); }

  /* Returns the sizes of all patches in the pool. */
  const std::vector<int>& patch_sizes() const { return num_pairs_; }

  /* Returns the number of velocities involved per patch. */
  const std::vector<int>& patch_num_velocities() const {
    return patch_num_velocities_;
  }

  /* Total number of pairs across all patches. */
  int total_num_pairs() const { return ssize(fn0_); }

  int num_pairs(int patch_index) const {
    DRAKE_ASSERT(0 <= patch_index && patch_index < num_patches());
    return num_pairs_[patch_index];
  }

  T& fn0(int pair) { return fn0_[pair]; }
  T& stiffness(int pair) { return stiffness_[pair]; }
  Matrix3<T>& R_WC(int pair) { return R_WC_[pair]; }
  Vector3<T>& p_AoC_W(int pair) { return p_AoC_W_[pair]; }
  Vector3<T>& p_BoC_W(int pair) { return p_BoC_W_[pair]; }

 private:
  // Pools of size num_patches().
  std::vector<int> num_pairs_;  // Number of pairs per patch.
  std::vector<std::array<int, 2>> cliques_;
  std::vector<std::array<int, 2>> cliques_nv_;
  std::vector<int> patch_num_velocities_;  // Num. velocities per patch.
  EigenPool<Vector6<T>> V_WA_;  // A's spatial velocity at current step.
  EigenPool<Vector6<T>> V_WB_;  // B's spatial velocity at current step.
  std::vector<T> dissipation_;       // Hunt & Crossley dissipation.
  std::vector<T> friction_;          // Friction coefficient.
  std::vector<double> stiction_tolerance_;
  std::vector<double> sigma_;

  // Pointers into the first column of the Jacobian columns pool. Of size
  // num_pairs() + 1. first_col_[pair_index] corresponds to the first column of
  // pair_index. first_col_[num_pairs()] corresponds to the total number of
  // non-zero columns.
  std::vector<int> JA_first_col_;
  std::vector<int> JB_first_col_;    
  EigenPool<Vector6<T>> J_WA_cols_;  // Spatial velocity Jacobian columns for A.
  EigenPool<Vector6<T>> J_WB_cols_;  // Spatial velocity Jacobian columns for B.  

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