#pragma once

#include <memory>
#include <set>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/fast_sap/sap_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

// Forward declare constraint types.

template <typename T>
using SapConstraintVariant =
    std::variant<SapPatchConstraint<T>, SapUnilateralConstraint<T>,
                 SapQuadraticCostConstraint<T>>;

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

  // Reserves memory for a fixed number of constraints.
  void ReserveContactConstraints(int num_constraints);
  void ReserveLimitConstraints(int num_constraints);

  // Resets quadratic and linear cost terms.
  // All contact constraints are removed and all other constraints remain.
  // Call AddConstraint() to incorporate new contact constraints.
  // @note No memory allocation is performed.
  void Reset(std::vector<MatrixX<T>> A, VectorX<T> r);

  /* The set of constraints is closed. These methods allow to add specific
   constraint types to the model.
   These methods do not allocate memory if the number of constraints is below
   the specified capacity with Reserve(). */
  int AddConstraint(SapPatchConstraint<T> constraint);

  /* Limit constraints are added on a per-clique basis. Therefore this method
   * can only be called at most num_cliques() times. */
  int AddConstraint(SapLimitConstraint<T> constraint);

  // TODO(amcastro-tri): Add these:
  //   SapActuationConstraint: actuation with effort limits. Per-clique.
  //   SapHolonomicConstraint: usually one or two cliques. Consider among more
  //   than two cliques?
  //   SapUnilateralConstraint: things like tendons. Maybe SapLimitConstraint
  //   covers this?

  /* Returns the number of cliques. */
  int num_cliques() const { return A_.size(); }

  int num_constraints() const {
    return 
  }

  /* Returns the total number of generalized velocities for this problem. */
  int num_velocities() const { return nv_; }

  /* Returns the total number of constraint equations. That is, nk = ∑ni where
   ni is the number of constraint equations for the i-th constraint, see
   SapConstraint::num_constraint_equations(). */
  int num_constraint_equations() const {
    return constraints_.size();
  }

  std::vector<int>& constraint_cliques(int constraint_index) const {
    return std::visit(
        [](auto& constraint) {
          return constraint.constraint_cliques();
        },
        constraints_[constraint_index]);
  }

  // Updates `data` as a function of v.
  void CalcData(const VectorX<T>& v, SapData<T>* data) const;

 private:
  // Input data provided at construction.
  T time_step_{0.0};           // Discrete time step.
  std::vector<MatrixX<T>> A_;  // Linear dynamics matrix.
  VectorX<T> r_;               // Cost linear term.

  // Data initialized at construction from input data.
  int nv_{0};  // Total number of generalized velocities.
#if 0  
  std::vector<int> velocities_start_;
  // Gives the index of the first constraint equation for each constraint.
  // Has size = num_constraints() + 1 and at any time:
  // constraint_equations_start_.back() == num_constraint_equations().
  std::vector<int> constraint_equations_start_{0};
#endif

  // Constraints. The set of possible constraints is closed.
  // N.B. This order is important. Do NOT change it.
  std::vector<SapConstraintVariant<T>> constraints_;
  //std::vector<SapPatchConstraint<T>> patch_constraints_;
  //std::vector<SapLimitConstraint<T>> limit_constraints_;
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
