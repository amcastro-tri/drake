#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/value.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/kernel/acceleration_kinematics_data.h"
#include "drake/multibody/kernel/multibody_kernel_parameters.h"
#include "drake/multibody/kernel/position_kinematics_data.h"
#include "drake/multibody/kernel/velocity_kinematics_data.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
class MultibodyKernel;

struct MobodInfo {
  MobodIndex index;
  MobodIndex inboard_index;
  std::vector<MobodIndex> outboard_indexes;
  int q_start;
  int v_start;
};

// N.B. This MobodBase class is useful to place generic code such as get_X_PF()
// and generic data such as "index()".
// We found out analyzing assembly generated code that, even if marking a
// function final, and even if the compiler knew the type at compiler time,
// making the function "virtual" was enough to hinder the compiler's ability to
// inline. Therefore we NEVER mark as "virtual"  any performance critical
// functions.
// Non critical functions like GetName() below can still be virtual if so
// needed.
template <typename T>
class MobodBase {
 public:
  using Scalar = T;

  MobodBase() = default;

  explicit MobodBase(MobodInfo info) : info_(std::move(info)){};

  int inboard_index() const { return info_.inboard_index; }
  int index() const { return info_.index; }
  const std::vector<MobodIndex>& outboards() const {
    return info_.outboard_indexes;
  }
  int first_position() const { return info_.q_start; }
  int first_velocity() const { return info_.v_start; }

  // Methods to access kernel parameters.
  const math::RigidTransform<T>& get_X_PF(
      const MultibodyKernelParameters<T>& parameters) const {
    return parameters.X_PF(index());
  }

  // Methods to access data.
  const math::RigidTransform<T>& get_X_PM(
      const PositionKinematicsData<T>& position_kinematics) const {
    return position_kinematics.X_PM(index());
  }

  math::RigidTransform<T>& get_mutable_X_PM(
      PositionKinematicsData<T>* data) const {
    return data->mutable_X_PM(index());
  }

  // Retrieves from `velocity_kinematics` the spatial velocity of `this` mobod's
  // parent frame P, expressed in P.
  // @pre `this` mobod is not the world mobod, which has no parent.
  const SpatialVelocity<T>& get_V_WP_P(
      const VelocityKinematicsData<T>& velocity_kinematics) const {
    return velocity_kinematics.V_WM_M(info_.inboard_index);
  }

  const SpatialVelocity<T>& get_V_WM_M(
      const VelocityKinematicsData<T>& vk) const {
    return vk.V_WM_M(info_.index);
  }

  // Returns a mutable reference to the spatial velocity of this mobod stored in
  // `velocity_kinematics`.
  SpatialVelocity<T>& get_mutable_V_WM_M(
      VelocityKinematicsData<T>* velocity_kinematics) const {
    return velocity_kinematics->mutable_V_WM_M(info_.index);
  }

  const SpatialAcceleration<T>& get_A_WP_P(
      const AccelerationKinematicsData<T>& ak) const {
    return ak.A_WM_M(info_.inboard_index);
  }

  SpatialAcceleration<T>& get_mutable_A_WM_M(
      AccelerationKinematicsData<T>* ak) const {
    return ak->mutable_A_WM_M(info_.index);
  }

 protected:
  /* Helper to pack `data` of type `U` into an AbstractValue. Derived constraint
   classes can use this helper to implement function to make data. */
  template <typename U>
  static std::unique_ptr<AbstractValue> MoveAndMakeAbstractValue(U&& data) {
    auto owned_data = std::make_unique<U>(std::move(data));
    return std::make_unique<Value<U>>(std::move(owned_data));
  }

 private:
  friend class MultibodyKernel<T>;

  MobodInfo info_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
