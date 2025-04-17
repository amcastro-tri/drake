#pragma once

#include <variant>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/fast_sap/sap_constraint_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

#if 0
template <typename T>
class SapDenseHessian {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapHessian);

  // Updates Hessian to H = A + Jᵀ⋅G⋅J, and factorizes it.
  void Update(const MatrixX<T>& A, const MatrixX<T>& J,
              const std::vector<MatrixX<T>>& G);

  // Solves v = H⁻¹⋅b.
  void Solve(const VectorX<T>& b, const MatrixX<T>* v) const;

 private:
  MatrixX<T> H_;
};
#endif

// Vector container of Eigen objects. The goal is to minimize and even avoid
// completely memory allocation when the size or structure of the vector
// changes.
// @tparam EigenType either MatrixX or VectorX.
template <typename EigenType>
class VectorOfEigenObjects {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VectorOfEigenObjects);

  // An empty vector of Eigen objects.
  VectorOfEigenObjectss() = default;

  // Resizes the array to store the provided sizes. Memory is only allocated if
  // strictly needed, when either capacity() or elements_capacity() is exceeded.
  void Resize(const std::vector<int>& sizes) {
    int num_elements = 0;
    for (int sz : sizes) {
      if constexpr (EigenType::ColsAtCompileTime == 1) {
        num_elements += sz;
      } else {
        num_elements += (sz * sz);
      }
    }
    storage_.resize(num_elements);
    maps_.clear();
    maps_.reserve(ssize(sizes));
    T* data = storage_.data();
    for (int sz : sizes) {
      if constexpr (EigenType::ColsAtCompileTime == 1) {
        maps_.push_back(Eigen::Map<EigenType>(data, sz));
        data += sz;
      } else {
        maps_.push_back(Eigen::Map<EigenType>(data, sz, sz));
        data += (sz * sz);
      }
    }
  }

  const Eigen::Map<const EigenType>& matrix(int i) {
    DRAKE_ASSERT(0 <= i < size());
    return maps_[i];
  }

  Eigen::Map<EigenType>& mutable_matrix(int i) {
    DRAKE_ASSERT(0 <= i < size());
    return maps_[i];
  }

  // Returns the number of matrices stored by this array.
  int size() const { return maps_.size(); }

  // Returns the total number of scalar elements stored as size()
  // matrices.
  int num_elements() const { return storage_.size(); }

  int capacity() { return maps_.capacity(); }
  int elements_capacity() { return maps_.capacity(); }

  std::vector<Eigen::Map<EigenType>>& mutable_maps() { return maps_; }

 private:
  std::vector<T> storage_;
  std::vector<Eigen::Map<EigenType>> maps_;
};

template <typename T>
using VectorOfVectorX = VectorOfEigenObjects<VectorX<T>>;
template <typename T>
using VectorOfMatrixX = VectorOfEigenObjects<MatrixX<T>>;

/* SAP generalized velocities v and SAP quantities function of v.

[Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An Unconstrained
Convex Formulation of Compliant Contact. Available at
https://arxiv.org/abs/2110.10107 */
template <typename T>
class SapData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapData);
  
  // @param clique_sizes The size of the c-th clique.
  // @param num_equations The number of equations of the k-th constraint.
  SapData(const std::vector<int>& clique_sizes,
          const std::vector<int>& num_equations)
      : constraints_workspace_(num_equations) {
    const int nv = std::accumulate(clique_sizes.begin(), clique_sizes.end(), 0);
    v_.resize(nv);
    cost_gradient_.resize(nv);
    cost_hessian_.resize(nv, nv);
  }

 private:
  // Generalized velocities of the model.
  // Everything in SapData is a function of v.
  VectorX<T> v_;

  T cost_;
  VectorX<T> cost_gradient_;
  // TODO(amcastro-tri): support a sparse Hessian.
  MatrixX<T> cost_hessian_;

  // Storage for constraints to work with.
  struct ConstraintsWorkspace {
    ConstraintsWorkspace(const std::vector<int>& num_equations) {
      const int num_constraints = ssize(num_equations);
      velocities.Resize(num_equations);
      impulses.Resize(num_equations);
      hessians.Resize(num_equations);
      G.Resize(num_equations);
      data.reserve(num_constraints);
      for (int k = 0; k < num_constraints; ++k) {
        data.emplace_back(&velocities[i], &impulses[i], &G[i]);
      }
    }

    int num_constraints() const { return data.size(); }

    VectorOfVectorX<T> velocities;  // Constraint velocities.
    VectorOfVectorX<T> impulses;
    VectorOfMatrixX<T> hessians;  // Hₖ = Jₖᵀ⋅Gₖ⋅Jₖ
    VectorOfMatrixX<T> G;  // Constraints Hessian, each Gₖ of size mₖ×mₖ.
    std::vector<SapConstraintData<T>> data;
  } constraints_workspace_;
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
