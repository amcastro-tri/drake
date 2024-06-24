#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/kernel/mobod_impl.h"
#include "drake/multibody/kernel/weld_mobod_data.h"
#include "drake/multibody/kernel/zero_spatial_velocity.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
class WeldMobod;

template <typename T>
struct MobodTraits<WeldMobod<T>> {
  using Scalar = T;
  static constexpr int num_configurations = 0;
  static constexpr int num_velocities = 0;

  using ConfigVector = Eigen::Vector<T, num_configurations>;
  using VelocitiesVector = Eigen::Vector<T, num_configurations>;
  using MobodRigidTransform = IdentityRigidTransform;
  using MobodSpatialMotion = ZeroSpatialMotion;
  using PositionKinematicsData = WeldMobodPositionKinematicsData;
  using VelocityKinematicsData = WeldMobodVelocityKinematicsData;
  using AccelerationKinematicsData = WeldMobodAccelerationKinematicsData;
};

template <typename T>
class WeldMobod : public MobodImpl<WeldMobod<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(WeldMobod);

  // TODO: Consider placing these in a macro so that we can reuse this code in
  // several mobods.
  using Base = MobodImpl<WeldMobod<T>>;
  using Traits = MobodTraits<WeldMobod>;
  static constexpr int num_configurations = Traits::num_configurations;
  static constexpr int num_velocities = Traits::num_velocities;
  using ConfigVector = typename Traits::ConfigVector;
  using VelocitiesVector = typename Traits::VelocitiesVector;
  using MobodSpatialMotion = typename Traits::MobodSpatialMotion;
  using PositionKinematicsData = typename Traits::PositionKinematicsData;
  using VelocityKinematicsData = typename Traits::VelocityKinematicsData;
  using AccelerationKinematicsData =
      typename Traits::AccelerationKinematicsData;

  WeldMobod() = default;

  explicit WeldMobod(MobodInfo info) : Base(std::move(info)){};

 private:
  /* Friend the base class so that MobodImpl::Foo() can reach its
   implementation, DoFoo(), in this class. */
  friend MobodImpl<WeldMobod>;

  // no-op.
  void DoProjectSpatialForce(const SpatialForce<T>&, VelocitiesVector*) const {}

  // No-op.
  void DoCalcPositionKinematicsData(const ConfigVector&,
                                    PositionKinematicsData*) const {}

  // No-op.
  void DoCalcVelocityKinematicsData(const ConfigVector&,
                                    const VelocitiesVector&,
                                    const PositionKinematicsData&,
                                    VelocityKinematicsData*) const {}

  // no-op.
  void DoCalcAccelerationKinematicsData(const ConfigVector&,
                                        const VelocitiesVector&,
                                        const VelocitiesVector&,
                                        const PositionKinematicsData&,
                                        const VelocityKinematicsData&,
                                        AccelerationKinematicsData*) const {}

  // no-op.
  void DoAccumulateAccelerationBias(const Vector3<T>&,
                                    const MobodSpatialMotion&,
                                    SpatialAcceleration<T>*) const {}
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
