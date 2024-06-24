#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_algebra.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
struct QuaternionFloatingMobodPositionKinematicsData {
  math::RigidTransform<T> X_FM;
};

template <typename T>
class QuaternionFloatingMobodSpatialMotion {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(QuaternionFloatingMobodSpatialMotion);

  // Zero spatial velocity.
  QuaternionFloatingMobodSpatialMotion() = default;

  // 6 DoF spatial motion.
  QuaternionFloatingMobodSpatialMotion(const Vector6<T>& V) : V_(V) {}

  const Vector6<T>& get_coeffs() const { return V_; }

  auto rotational() const { return V_.template head<3>(); }

  auto translational() const { return V_.template tail<3>(); }

 private:
  // and are expressed in the same frame, W.
  template <template <typename> class SpatialMotion, typename U>
  friend SpatialMotion<U> operator+(
      const SpatialMotion<U>& V_WPb,
      const QuaternionFloatingMobodSpatialMotion<U>& V_PB_W);

  Vector6<T> V_{Vector6<T>::Zero()};
};

// Compose spatial velocities as V_WB = V_WPb + V_PB_W.
// @pre both operands contribute to the spatial velocity of the same point, B,
// and are expressed in the same frame, W.
template <template <typename> class SpatialMotion, typename T>
SpatialMotion<T> operator+(
    const SpatialMotion<T>& V_WPb,
    const QuaternionFloatingMobodSpatialMotion<T>& V_PB_W) {
  static_assert(std::is_same_v<SpatialMotion<T>, SpatialVelocity<T>> ||
                std::is_same_v<SpatialMotion<T>, SpatialAcceleration<T>>);
  SpatialMotion<T> V_WB(V_WPb);
  V_WB.get_coeffs() += V_PB_W.get_coeffs();
  return V_WB;
}

template <typename T>
struct QuaternionFloatingMobodVelocityKinematicsData {
  QuaternionFloatingMobodSpatialMotion<T> V_FM;
};

template <typename T>
struct QuaternionFloatingMobodAccelerationKinematicsData {
  QuaternionFloatingMobodSpatialMotion<T> A_FM_M;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
