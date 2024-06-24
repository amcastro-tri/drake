#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/value.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/kernel/mobod_data_variant.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
class PositionKinematicsData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionKinematicsData);

  PositionKinematicsData(
      std::vector<MobodPositionKinematicsDataVariant<T>> mobod_data)
      : mobod_data_(std::move(mobod_data)) {
    const int num_mobods = ssize(mobod_data_);
    X_PM_.resize(num_mobods);
  }

  const std::vector<math::RigidTransform<T>>& X_PM() const { return X_PM_; }

  // Returns mobod type-specific forward kinematics data for the i-th mobod.
  const MobodPositionKinematicsDataVariant<T>& mobod_data(int i) const {
    return mobod_data_[i];
  }

  // Returns mobod type-specific forward kinematics data for the i-th mobod.
  MobodPositionKinematicsDataVariant<T>& mutable_mobod_data(int i) {
    return mobod_data_[i];
  }

  const math::RigidTransform<T>& X_PM(int i) const { return X_PM_[i]; }

  math::RigidTransform<T>& mutable_X_PM(int i) { return X_PM_[i]; }

 private:
  // For the i-th mobod B, X_PB_[i] stores the pose of B on the parent mobod P.
  std::vector<math::RigidTransform<T>> X_PM_;

  // Mobod type-specific forward kinematics data.
  std::vector<MobodPositionKinematicsDataVariant<T>> mobod_data_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
