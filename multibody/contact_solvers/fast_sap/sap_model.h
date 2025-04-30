#pragma once

#include <memory>
#include <set>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/fast_sap/eigen_pool.h"
#include "drake/multibody/contact_solvers/fast_sap/patch_constraint.h"
#include "drake/multibody/contact_solvers/fast_sap/sap_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

/* A model of the SAP problem.
 It models the discrete momentum equation:
    A⋅v = r + Jᵀ⋅γ,
 which corresponds to the convex cost:
    ℓ(v) = 1/2‖v‖²−r⋅v + ℓ(vc). 

[Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An Unconstrained
Convex Formulation of Compliant Contact. Available at
https://arxiv.org/abs/2110.10107 */
template <typename T>
class SapModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapModel);

  // Constructor for an empty model.
  SapModel() = default;

  /* Resets quadratic and linear cost terms.
   All contact constraints are removed and all other constraints remain.
   Call AddConstraint() to incorporate new contact constraints.
   @note No memory allocation is performed. */
  // TODO(amcastro-tri): Consider a mechanism to avoid copying this data.
  // Something like:
  //   1. Reset: Reset(time_step, const std::vector<int>& clique_sizes)
  //   1b. set_time_step(): when problem size does not change.
  //   2. mutable accessors: get_A(), get_r(), and write directly into them.
  void Reset(double time_step, const EigenPool<MatrixX<T>>& A,
             const VectorX<T>& r) {
    DRAKE_ASSERT(time_step > 0);
    time_step_ = time_step;
    A_ = A;
    num_velocities_ = 0;
    clique_sizes_.reserve(A.size());
    for (int c = 0; c < A.size(); ++c) {
      DRAKE_ASSERT(A[c].rows() == A[c].cols());
      const int clique_nv = A[c].rows();
      num_velocities_ += clique_nv;
      clique_sizes_.push_back(clique_nv);
    }
    r_ = r;
    DRAKE_ASSERT(r_.size() == num_velocities_);
    patch_constraints_.clear();
    patch_params_pool_.Clear();
  }

  /* Returns the number of cliques. */
  int num_cliques() const { return clique_sizes_.size(); }

  const std::vector<int>& clique_sizes() const { return clique_sizes_; }

  /* Returns the total number of generalized velocities for this problem. */
  int num_velocities() const { return num_velocities_; }

  /* Total number of constraints. */
  int num_constraints() const { return num_patch_constraints(); }

  /* Total number of constraint equations. */
  int num_constraint_equations() const {
    return patch_params_pool_.num_constraint_equations();
  }

  /* Adds a patch constraint.

    @param cliques clique[0] and clique[1] provide the clique index for body A
    and B, respectively. The clique index is negative if the body is anchored,
    and thus its Jacobian has zero size. 

    @note cliques[0] < 0 implies J_WA.size() = 0.
    @note cliques[1] < 0 implies J_WB.size() = 0.

    @returns patch constraint index. */
  int AddPatchConstraint(
      std::array<int, 2> cliques, const Vector6<T>& V_WA0,
      const Matrix6X<T>& J_WA, const Vector6<T>& V_WB0, const Matrix6X<T>& J_WB,
      const T& dissipation, const T& friction, double vs,
      double sigma /*, PatchConstraintApproximation::kLagged */) {
    const int patch_index = num_patch_constraints();
    patch_constraints_.emplace_back(patch_index);
    patch_params_pool_.PushBackPatchData(cliques, V_WA0, J_WA, V_WB0, J_WB,
                                         dissipation, friction, vs, sigma);
    return patch_index;
  }

  /* Adds contact pair to the last patch constraint added with
  AddPatchConstraint(). */
  void AddPatchPair(const T& fn0, const T& stiffness, const Matrix3<T>& R_WC,
                    const Vector3<T>& p_AoC_W, const Vector3<T>& p_BoC_W) {
    patch_params_pool_.PushBackPairData(fn0, stiffness, R_WC, p_AoC_W, p_BoC_W);
  }

  PatchConstraintParamsPool<T>& patch_constraint_params() {
    return patch_params_pool_;
  }

  /* Clears patch constraints. No memory is freed. */
  void ClearPatchConstraints() {
    patch_constraints_.clear();
    patch_params_pool_.Clear();
  }

  /* Limit constraints are added on a per-clique basis. Therefore this method
   * can only be called at most num_cliques() times. */
  //int AddConstraint(SapLimitConstraint<T> constraint);

  // TODO(amcastro-tri): Add these:
  //   SapActuationConstraint: actuation with effort limits. Per-clique.
  //   SapHolonomicConstraint: usually one or two cliques. Consider among more
  //   than two cliques?
  //   SapUnilateralConstraint: things like tendons. Maybe SapLimitConstraint
  //   covers this?  

  int num_patch_constraints() const { return ssize(patch_constraints_); }  

  /* Resizes data accordingly to store data for this model.
   No allocations are required if data's capacity is already enough. */
  void ResizeData(SapData<T>* data) const {
    data->Resize(num_velocities_, clique_sizes_,
                 patch_params_pool_.patch_sizes(),
                 patch_params_pool_.patch_num_velocities());
  }

  // Updates `data` as a function of v.
  //void CalcData(const VectorX<T>& v, SapData<T>* data) const;

 private:
  // Input data provided at construction.
  T time_step_{0.0};           // Discrete time step.  
  EigenPool<MatrixX<T>> A_;  // Linear dynamics matrix.
  VectorX<T> r_;               // Cost linear term

  // Total number of generalized velocities. = sum(clique_sizes_).
  int num_velocities_{0};  
  std::vector<int> clique_sizes_;  // Number of velocities per patch.

  std::vector<PatchConstraint<T>> patch_constraints_;
  PatchConstraintParamsPool<T> patch_params_pool_;
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
