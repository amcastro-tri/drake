#pragma once

#include <variant>

#include "drake/multibody/kernel/quaternion_floating_mobod.h"
#include "drake/multibody/kernel/revolute_mobod.h"
#include "drake/multibody/kernel/weld_mobod.h"
#include "drake/multibody/kernel/world_mobod.h"

namespace drake {
namespace multibody {
namespace internal {

#if 0
template <typename T>
using MobodVariant =
    std::variant<RevoluteMobodX<T>, RevoluteMobodZ<T>, RevoluteMobodY<T>>;
#endif

template <typename T>
using MobodVariant =
    std::variant<WeldMobod<T>, WorldMobod<T>, RevoluteMobod<T, 0>,
                 RevoluteMobod<T, 1>, RevoluteMobod<T, 2>,
                 QuaternionFloatingMobod<T>>;
}  // namespace internal
}  // namespace multibody
}  // namespace drake
