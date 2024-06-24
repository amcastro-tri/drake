#pragma once

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/value.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
struct MobodParameters {
  math::RigidTransform<T> X_PF;
  // Even though all kernel computations are performed in the mobilized frame M,
  // we still allow users to ask questions about a different body frame B, at a
  // fixed pose from its inboard handle M.
  math::RigidTransform<T> X_MB;
  multibody::SpatialInertia<T> M_BMo_M;
};

template <typename T>
class MultibodyKernelParameters {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyKernelParameters);

  // Constructs a set of parameters with capacity for a kernel with `num_mobods`
  // mobilized bodies. Mobilized body with index 0 corresponds to the world.
  // Parameters for he world are never initialized to valid values and should
  // never be used in computations.
  MultibodyKernelParameters(int num_mobods) {
    X_PF_.reserve(num_mobods);
    M_BMo_M_.reserve(num_mobods);
    // Garbage values for the world parameters, which must never be used.
    X_PF_.push_back(math::RigidTransform<T>());
    X_MB_.push_back(math::RigidTransform<T>());
    M_BMo_M_.push_back(multibody::SpatialInertia<T>::NaN());
  }

  void AddParameters(int mobod_index, MobodParameters<T> p) {
    DRAKE_DEMAND(ssize(X_PF_) == mobod_index);
    DRAKE_DEMAND(ssize(M_BMo_M_) == mobod_index);
    X_PF_.push_back(std::move(p.X_PF));
    X_MB_.push_back(std::move(p.X_MB));
    M_BMo_M_.push_back(std::move(p.M_BMo_M));
  }

  const std::vector<math::RigidTransform<T>>& get_X_PF() const { return X_PF_; }

  // Returns pose of the "fixed" frame F, inboard across the its mobilizer, to
  // the i-th mobod B.
  const math::RigidTransform<T>& X_PF(int i) const { return X_PF_[i]; }

  const math::RigidTransform<T>& X_MB(int i) const { return X_MB_[i]; }

  const multibody::SpatialInertia<T>& M_BMo_M(int i) const {
    return M_BMo_M_[i];
  }

 private:
  // For the i-th mobod B, X_PF_[i] stores the pose of the rigidly attached
  // "fixed" frame F on the parent mobod P.
  std::vector<math::RigidTransform<T>> X_PF_;

  std::vector<math::RigidTransform<T>> X_MB_;

  // Spatial inertia for the i-th body B, about the origin of the inboard
  // mobilized frame M, and expressed in M.
  std::vector<multibody::SpatialInertia<T>> M_BMo_M_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
