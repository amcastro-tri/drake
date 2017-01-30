#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"

namespace drake {
namespace multibody {

/// Right now a dummy class to avoid dependency on systems::Context<T>.
///
template <typename T>
class MultibodyTreeContext {
 public:

  const VectorX<T>& get_positions() const { return q_; }
  const VectorX<T>& get_velocities() const { return v_; }

  PositionKinematicsCache<T>& get_mutable_position_kinematics() const {
    return position_kinematics_;
  }

  /// Returns the current time in seconds.
  const T& get_time() const { return time_sec_; }
 private:
  T time_sec_;
  VectorX<T> q_, v_;
  mutable PositionKinematicsCache<T> position_kinematics_;
};

}  // namespace multibody
}  // namespace drake
