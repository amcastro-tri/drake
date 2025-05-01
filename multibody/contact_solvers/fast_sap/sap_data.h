#pragma once

#include <variant>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/fast_sap/patch_constraint_data_pool.h"

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

  struct Cache {
    void Resize(int num_velocities, const std::vector<int>& patch_sizes,
                const std::vector<int>& patch_num_velocities) {
      const int nv = num_velocities;
      Av.resize(nv);
      cost_gradient.resize(nv);
      cost_hessian.resize(nv, nv);
      patch_constraints_data.Resize(patch_sizes, patch_num_velocities);
    }    

    T momentum_cost{0};
    T constraints_cost{0};
    T cost;
    VectorX<T> Av;             // = A * v.
    VectorX<T> cost_gradient;  // Of size num_velocities().
    MatrixX<T> cost_hessian;   // Square matrix of size num_velocities().

    // Type-specific constraint pools.
    PatchConstraintDataPool<T> patch_constraints_data;
  };

  /* Struct to store pre-allocated scratch space.
   This space is not intended for long-term storage and is often cleared or
   overwritten as needed. */
  struct Scratch {
    // TODO: consider using an EigenPool here since pools support the concept of
    // "capacity", while VectorX does not.

    // Vectors of size PooledSapModel::num_velocities().
    VectorX<T> v1;
    VectorX<T> v2;
    VectorX<T> v3;
  };

  /* Default constructor for empty data. */
  SapData() = default;

  /* @param num_velocities Total number of generalized velocities.
     @param patch_sizes Number of contact pairs for each patch.
     @param patch_num_velocities Number of participating velocities per patch.
     */
  void Resize(int num_velocities, const std::vector<int>& clique_sizes,
              const std::vector<int>& patch_sizes,
              const std::vector<int>& patch_num_velocities) {
    v_.resize(num_velocities);
    cache_.Resize(num_velocities, patch_sizes, patch_num_velocities);
  }

  int num_velocities() const { return v_.size(); }

  int num_patches() const {
    return cache_.patch_constraints_data.num_patches();
  }

  const VectorX<T>& v() const { return v_; }
  VectorX<T>& v() { return v_; }

  const Cache& cache() const { return cache_; }
  Cache& cache() { return cache_; }

  Scratch& scratch() const { return scratch_; }

 private:
  VectorX<T> v_;  // Generalized velocities.
  Cache cache_;   // All other quantities function of v.
  // We allow PooledSapModel methods to write on the scratch as needed.
  mutable Scratch scratch_;
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
