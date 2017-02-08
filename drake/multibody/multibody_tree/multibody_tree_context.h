#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"
#include "drake/multibody/multibody_tree/composite_body_inertias_cache.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

/// Right now a dummy class to avoid dependency on systems::Context<T>.
///
template <typename T>
class MultibodyTreeContext {
 public:
  MultibodyTreeContext(const MultibodyTreeTopology& tree_topology) {
    // Allocate state data.
    q_.resize(tree_topology.num_positions);
    v_.resize(tree_topology.num_velocities);

    // Allocate cache entries.
    position_kinematics_.Allocate(tree_topology);
    composite_body_inertias_.Allocate(tree_topology);
  }

  const VectorX<T>& get_positions() const { return q_; }
  const VectorX<T>& get_velocities() const { return v_; }

  VectorX<T>& get_mutable_positions() { return q_; }
  VectorX<T>& get_mutable_velocities() { return v_; }

  const PositionKinematicsCache<T>& get_position_kinematics() const {
    return position_kinematics_;
  }

  PositionKinematicsCache<T>* get_mutable_position_kinematics() const {
    return &position_kinematics_;
  }

  const CompositeBodyInertiasCache<T>& get_cbi_cache() const {
    return composite_body_inertias_;
  }

  CompositeBodyInertiasCache<T>* get_mutable_cbi_cache() const {
    return &composite_body_inertias_;
  }

  /// Returns the current time in seconds.
  const T& get_time() const { return time_sec_; }

  void Print() {
    PRINT_VAR(q_.transpose());
    PRINT_VAR(v_.transpose());
    position_kinematics_.Print();
    composite_body_inertias_.Print();
  }

 private:
  T time_sec_;
  VectorX<T> q_, v_;
  // Cached entries.
  mutable PositionKinematicsCache<T> position_kinematics_;
  mutable CompositeBodyInertiasCache<T> composite_body_inertias_;
};

}  // namespace multibody
}  // namespace drake
