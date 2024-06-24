#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/value.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/kernel/mobod_data_variant.h"
#include "drake/multibody/math/spatial_algebra.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
class VelocityKinematicsData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VelocityKinematicsData);

  VelocityKinematicsData(
      std::vector<MobodVelocityKinematicsDataVariant<T>> mobod_data)
      : mobod_data_(std::move(mobod_data)) {
    const int num_mobods = ssize(mobod_data_);
    V_WM_M_.resize(num_mobods);
    V_WM_M_[0].SetZero();  // The world's velocity is forever zero.
  }

  // Returns mobod type-specific forward kinematics data for the i-th mobod.
  const MobodVelocityKinematicsDataVariant<T>& mobod_data(int i) const {
    return mobod_data_[i];
  }

  MobodVelocityKinematicsDataVariant<T>& mutable_mobod_data(int i) {
    return mobod_data_[i];
  }

  const SpatialVelocity<T>& V_WM_M(int i) const { return V_WM_M_[i]; }

  SpatialVelocity<T>& mutable_V_WM_M(int i) { return V_WM_M_[i]; }

 private:
  // For the i-th mobod body B, V_WM_M_[i] stores the spatial velocity of its
  // inboard (or "mobilized") frame M measured in the world frame W and
  // expressed in the mobilized frame M. Expressing V_WM in M is convenient so
  // that the equations of motion for inverse dynamics can be expressed in M,
  // resulting on body spatial forces expressed directly in frame M, F_BMo_M.
  // Therefore projecting forces can be very well optimized for axis aligned
  // mobilizers.
  std::vector<SpatialVelocity<T>> V_WM_M_;

  // Mobod type-specific forward kinematics data.
  std::vector<MobodVelocityKinematicsDataVariant<T>> mobod_data_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
