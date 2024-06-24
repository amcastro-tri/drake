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

class WorldRigidTransform {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(WorldRigidTransform);

  // Identity transform.
  WorldRigidTransform() = default;

 private:
  template <typename U>
  friend math::RigidTransform<U> operator*(const math::RigidTransform<U>& X_AB,
                                           const WorldRigidTransform);
};

// Overload to comply with MobodImpl's contract. The pose of the world body is
// not defined, and operating with it is not allowed. This method always throws.
template <typename T>
math::RigidTransform<T> operator*(const math::RigidTransform<T>&,
                                  const WorldRigidTransform) {
  throw std::logic_error("Do not operate with the pose of the 'world'");
  DRAKE_UNREACHABLE();
}

struct WorldMobodPositionKinematicsData {
  WorldRigidTransform X_FM;
};

struct WorldMobodVelocityKinematicsData {
  ZeroSpatialMotion V_FM;
};

struct WorldMobodAccelerationKinematicsData {
  ZeroSpatialMotion A_FM_M;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
