#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/kernel/mobod_impl.h"
#include "drake/multibody/kernel/world_mobod_data.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
class WorldMobod;

template <typename T>
struct MobodTraits<WorldMobod<T>> {
  using Scalar = T;
  static constexpr int num_configurations = 0;
  static constexpr int num_velocities = 0;

  using ConfigVector = Eigen::Vector<T, num_configurations>;
  using VelocitiesVector = Eigen::Vector<T, num_configurations>;
  using MobodRigidTransform = WorldRigidTransform;
  using MobodSpatialMotion = ZeroSpatialMotion;
  using PositionKinematicsData = WorldMobodPositionKinematicsData;
  using VelocityKinematicsData = WorldMobodVelocityKinematicsData;
  using AccelerationKinematicsData = WorldMobodAccelerationKinematicsData;
};

template <typename T>
class WorldMobod : public MobodImpl<WorldMobod<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(WorldMobod);

  // TODO: Consider placing these in a macro so that we can reuse this code in
  // several mobods.
  using Base = MobodImpl<WorldMobod<T>>;
  using Traits = MobodTraits<WorldMobod>;
  static constexpr int num_configurations = Traits::num_configurations;
  static constexpr int num_velocities = Traits::num_velocities;
  using ConfigVector = typename Traits::ConfigVector;
  using VelocitiesVector = typename Traits::VelocitiesVector;
  using MobodSpatialMotion = typename Traits::MobodSpatialMotion;
  using PositionKinematicsData = typename Traits::PositionKinematicsData;
  using VelocityKinematicsData = typename Traits::VelocityKinematicsData;
  using AccelerationKinematicsData =
      typename Traits::AccelerationKinematicsData;

  WorldMobod() = default;

 private:
  /* Friend the base class so that MobodImpl::Foo() can reach its
   implementation, DoFoo(), in this class. */
  friend MobodImpl<WorldMobod>;

  void DoProjectSpatialForce(const SpatialForce<T>&, VelocitiesVector*) const {
    throw std::logic_error(
        "ProjectSpatialForce() should never be called on the 'world' "
        "mobilized body.");
  }

  // Throws. It should never be called on the world mobod.
  void DoCalcPositionKinematicsData(const ConfigVector&,
                                    PositionKinematicsData*) const {
    throw std::logic_error(
        "CalcPositionKinematicsData() should never be called on the 'world' "
        "mobilized body.");
  }

  // Throws. It should never be called on the world mobod.
  void DoCalcVelocityKinematicsData(const ConfigVector&,
                                    const VelocitiesVector&,
                                    const PositionKinematicsData&,
                                    VelocityKinematicsData*) const {
    throw std::logic_error(
        "CalcPositionKinematicsData() should never be called on the 'world' "
        "mobilized body.");
  }

  void DoCalcAccelerationKinematicsData(
      const ConfigVector&, const VelocitiesVector&, const VelocitiesVector&,
      const PositionKinematicsData&, const VelocityKinematicsData&,
      AccelerationKinematicsData*) const {
    throw std::logic_error(
        "CalcAccelerationKinematicsData() should never be called on the "
        "'world' mobilized body.");
  }

  void DoAccumulateAccelerationBias(const Vector3<T>&,
                                    const MobodSpatialMotion&,
                                    SpatialAcceleration<T>*) const {
    throw std::logic_error(
        "AccumulateAccelerationBias() should never be called on the "
        "'world' mobilized body.");
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
