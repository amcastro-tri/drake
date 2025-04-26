#pragma once

#include <variant>
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



/* SAP generalized velocities v and SAP quantities function of v.

[Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An Unconstrained
Convex Formulation of Compliant Contact. Available at
https://arxiv.org/abs/2110.10107 */
template <typename T>
class SapData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapData);

  // @param clique_sizes Number of velocities for the c-th clique.
  // @param patch_sizes Size of each patch constraint.
  // @param num_velocities Number of velocities involved in .
  SapData(const SapModelSizes& model_sizes) {
    const int nv = model_sizes.num_velocities;
    v_.resize(nv);
    cost_gradient_.resize(nv);
    cost_hessian_.resize(nv);

    patch_constraints_pool_.Resize(model_sizes.patch_sizes,
                                   model_sizes.constraints.num_velocities);
  }

  int num_velocities() const { return v_.size(); }

 private:
  // Generalized velocities of the model.
  // Everything in SapData is a function of v.
  VectorX<T> v_;

  T cost_;
  VectorX<T> cost_gradient_;  // Of size num_velocities().
  // TODO(amcastro-tri): support a sparse Hessian.
  MatrixX<T> cost_hessian_;  // Square matrix of size num_velocities().

  // Type-specific constraint pools.
  PatchConstraintDataPool<T> patch_constraints_pool_;
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
