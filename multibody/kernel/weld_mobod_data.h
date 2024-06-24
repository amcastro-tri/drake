#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/kernel/zero_spatial_velocity.h"

namespace drake {
namespace multibody {
namespace internal {
class IdentityRigidTransform {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IdentityRigidTransform);

  // Identity transform.
  IdentityRigidTransform() = default;

 private:
  template <typename U>
  friend math::RigidTransform<U> operator*(const math::RigidTransform<U>& X_AB,
                                           const IdentityRigidTransform);
};

// Trivial case for X_AC = X_AB * Id = X_AB.
template <typename T>
math::RigidTransform<T> operator*(const math::RigidTransform<T>& X_AB,
                                  const IdentityRigidTransform) {
  return X_AB;
}

struct WeldMobodPositionKinematicsData {
  IdentityRigidTransform X_FM;
};

struct WeldMobodVelocityKinematicsData {
  ZeroSpatialMotion V_FM;
};

struct WeldMobodAccelerationKinematicsData {
  ZeroSpatialMotion A_FM_M;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
