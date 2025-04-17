#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

// Provides aliases into data for a specific constraint.
template <typename T>
class SapConstraintData {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapConstraintData);

  SapConstraintData(Eigen::Map<VectorX<T>>* vc, Eigen::Map<VectorX<T>>* gamma,
                    Eigen::Map<VectorX<T>>* G)
      : vc_(vc), gamma_(gamma), G_(G) {
    DRAKE_ASSERT(vc != nullptr);
    DRAKE_ASSERT(gamma != nullptr);
    DRAKE_ASSERT(G != nullptr);
    DRAKE_ASSERT(vc->size() == gamma->size());
    DRAKE_ASSERT(G->rows() == gamma->size());
    DRAKE_ASSERT(G->cols() == gamma->size());
  }

  int num_equations() const { return vc->size(); }

  const Eigen::Map<const VectorX<T>>& get_vc() { return *vc_; }
  const Eigen::Map<VectorX<T>>& get_mutable_vc() { return *vc_; }

#if 0
  Eigen::Map<const VectorX<T>> get_gamma() const {
    return as_eigen(gamma_, num_equations_);
  }

  Eigen::Map<VectorX<T>> get_mutable_gamma() {
    return as_mutable_eigen(gamma_, num_equations_);
  }

  Eigen::Map<const MatrixX<T>> get_H() const {
    return as_eigen(H_, num_velocities_, num_velocities_);
  }

  Eigen::Map<const MatrixX<T>> get_mutable_H() const {
    return as_mutable_eigen(H_, num_velocities_, num_velocities_);
  }
#endif

 private:
  T cost_{0.0};
  Eigen::Map<VectorX<T>>* vc_{nullptr};
  Eigen::Map<VectorX<T>>* gamma_{nullptr};
  Eigen::Map<MatrixX<T>>* G_{nullptr};
  //Eigen::Map<MatrixX<T>>& H_;

  // Hc = Jᵀ⋅G(vc)⋅J.
  // For a constraint k that constrains nₖ velocities, Hc has size nₖ×nₖ.
  // nₖ = ∑ nc, where nc is the numer of velocities of the c-th clique.
  //T* H_;

#if 0
  static Eigen::Map<const VectorX<T>> as_eigen(T* data, int size) {
    return Eigen::Map<const VectorX<T>>(data, size);
  }

  static Eigen::Map<VectorX<T>> as_mutable_eigen(T* data, int size) {
    return Eigen::Map<VectorX<T>>(data, size);
  }

  static Eigen::Map<const MatrixX<T>> as_eigen(T* data, int rows, int cols) {
    return Eigen::Map<const MatrixX<T>>(data, rows, cols);
  }

  static Eigen::Map<MatrixX<T>> as_mutable_eigen(T* data, int rows, int cols) {
    return Eigen::Map<MatrixX<T>>(data, rows, cols);
  }
#endif
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
