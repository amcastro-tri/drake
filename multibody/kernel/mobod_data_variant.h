#pragma once

#include <variant>

#include "drake/multibody/kernel/revolute_mobod_data.h"
#include "drake/multibody/kernel/weld_mobod_data.h"
#include "drake/multibody/kernel/world_mobod_data.h"
#include "drake/multibody/kernel/quaternion_floating_mobod_data.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
using MobodPositionKinematicsDataVariant =
    std::variant<WeldMobodPositionKinematicsData,
                 WorldMobodPositionKinematicsData,
                 RevoluteMobodPositionKinematicsData<T, 0>,
                 RevoluteMobodPositionKinematicsData<T, 1>,
                 RevoluteMobodPositionKinematicsData<T, 2>,
                 QuaternionFloatingMobodPositionKinematicsData<T>>;

template <typename T>
using MobodVelocityKinematicsDataVariant =
    std::variant<WeldMobodVelocityKinematicsData,
                 WorldMobodVelocityKinematicsData,
                 RevoluteMobodVelocityKinematicsData<T, 0>,
                 RevoluteMobodVelocityKinematicsData<T, 1>,
                 RevoluteMobodVelocityKinematicsData<T, 2>,
                 QuaternionFloatingMobodVelocityKinematicsData<T>>;

template <typename T>
using MobodAccelerationKinematicsDataVariant =
    std::variant<WeldMobodAccelerationKinematicsData,
                 WorldMobodAccelerationKinematicsData,
                 RevoluteMobodAccelerationKinematicsData<T, 0>,
                 RevoluteMobodAccelerationKinematicsData<T, 1>,
                 RevoluteMobodAccelerationKinematicsData<T, 2>,
                 QuaternionFloatingMobodAccelerationKinematicsData<T>>;

}
}  // namespace multibody
}  // namespace drake
