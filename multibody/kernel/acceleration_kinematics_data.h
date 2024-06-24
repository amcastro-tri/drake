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
class AccelerationKinematicsData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AccelerationKinematicsData);

  AccelerationKinematicsData(
      std::vector<MobodAccelerationKinematicsDataVariant<T>> mobod_data)
      : mobod_data_(std::move(mobod_data)) {
    const int num_mobods = ssize(mobod_data_);
    A_WM_M_.resize(num_mobods);
    // The world's acceleration.
    // N.B. We might set this to -g for inverse dynamics.
    A_WM_M_[0].SetZero();
  }

  // Returns mobod type-specific forward kinematics data for the i-th mobod.
  const MobodAccelerationKinematicsDataVariant<T>& mobod_data(int i) const {
    return mobod_data_[i];
  }

  MobodAccelerationKinematicsDataVariant<T>& mutable_mobod_data(int i) {
    return mobod_data_[i];
  }

  const SpatialAcceleration<T>& A_WM_M(int i) const { return A_WM_M_[i]; }

  SpatialAcceleration<T>& mutable_A_WM_M(int i) { return A_WM_M_[i]; }

 private:
  // Spatial accelerations of the mobilized frames M, measured in the (inertial)
  // world frame, and expressed in M.
  std::vector<SpatialAcceleration<T>> A_WM_M_;

  // Mobod type-specific data.
  std::vector<MobodAccelerationKinematicsDataVariant<T>> mobod_data_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
