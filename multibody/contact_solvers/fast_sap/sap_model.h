#pragma once

#include <memory>
#include <set>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/fast_sap/patch_constraint_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

/* A model of the SAP problem.

[Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An Unconstrained
Convex Formulation of Compliant Contact. Available at
https://arxiv.org/abs/2110.10107 */
template <typename T>
class SapModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapModel);

  /* Constructs a SAP model for the discrete momentum equation:
       A⋅v = r + Jᵀ⋅γ,
     which corresponds to the convex cost:
      ℓ(v) = 1/2‖v‖²−r⋅v + ℓ(vc). */
  SapModel(double time_step, std::vector<MatrixX<T>> A, VectorX<T> r);

  // Resets quadratic and linear cost terms.
  // All contact constraints are removed and all other constraints remain.
  // Call AddConstraint() to incorporate new contact constraints.
  // @note No memory allocation is performed.
  void Reset(double time_step, std::vector<MatrixX<T>> A, VectorX<T> r);  

  /* Adds a patch constraint.

    @warning We must call AddPir() on the returned view as many times needed
    before calling the next AddPatchConstraint().

    TODO: can we enforce this??? some sort of guard in the view? so that when it
    goes out of scope it tells the pool we are ready for a new patch?


    @param cliques clique[0] and clique[1] provide the clique index for body A
    and B, respectively. The clique index is negative if the body is anchored,
    and thus its Jacobian has zero size. */
  PatchConstraintParamsView<T> AddPatchConstraint(
      std::array<int, 2> cliques, const Matrix6X<T>& J_WA,
      const Matrix6X<T>& J_WB, const T& dissipation, const T& friction,
      double vs, double sigma /*, PatchConstraintApproximation::kLagged */) {
    const int patch_index = patch_params_pool_.num_patches();
    const int pair_index = patch_params_pool_.num_pairs();

    patch_params_pool_.Add(cliques, J_WA, J_WB, dissipation, friction, vs,
                           sigma);

    return PatchConstraintParamsView<T>(patch_index, pair_index,
                                        &patch_params_pool_);
  }

  const PatchConstraintParamsPool<T>& patch_constraint_params() {
    return patch_params_pool_;
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

  /* Returns the number of cliques. */
  int num_cliques() const { return A_.size(); }

  /* Total number of constraints. */
  int num_constraints() const { return patch_params_.num_constraints(); }

  /* Total number of constraint equations. */
  int num_constraint_equations() const {
    return patch_params_.num_constraint_equations();
  }

  /* Returns the total number of generalized velocities for this problem. */
  int num_velocities() const { return nv_; }

  /* Returns the total number of constraint equations. That is, nk = ∑ni where
   ni is the number of constraint equations for the i-th constraint, see
   SapConstraint::num_constraint_equations(). */
  int num_constraint_equations() const {
    return constraints_.size();
  }

  // Updates `data` as a function of v.
  //void CalcData(const VectorX<T>& v, SapData<T>* data) const;

 private:
  ModelSizes sizes;

  // Input data provided at construction.
  T time_step_{0.0};           // Discrete time step.
  std::vector<MatrixX<T>> A_;  // Linear dynamics matrix.
  VectorX<T> r_;               // Cost linear term.

  PatchConstraintParamsPool<T> patch_params_pool_;
  // PatchConstraintPool<T> patch_constraints_; most likely just std::vector<PatchConstraint<T>> 
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
