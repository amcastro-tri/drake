#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"

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

#if 0
    // Setup per-mobilizer contexts.
    mobilizer_contexts_.reserve(tree_topology.get_num_mobilizers());
    for (MobilizerIndex mobilizer_id(0);
         mobilizer_id < tree_topology.get_num_mobilizers(); ++mobilizer_id) {
      mobilizer_contexts_.push_back(
          CreateMobilizerContext(tree_topology, mobilizer_id));
    }
#endif
  }

  const VectorX<T>& get_positions() const { return q_; }
  const VectorX<T>& get_velocities() const { return v_; }

  VectorX<T>& get_mutable_positions() { return q_; }
  VectorX<T>& get_mutable_velocities() { return v_; }

  PositionKinematicsCache<T>* get_mutable_position_kinematics() const {
    return &position_kinematics_;
  }

#if 0
  const MobilizerContext<T>& get_mobilizer_context(
      MobilizerIndex mobilizer_id) const
  {
    return mobilizer_contexts_[mobilizer_id];
  }

  MobilizerContext<T>& get_mutable_mobilizer_context(
      MobilizerIndex mobilizer_id)
  {
    return mobilizer_contexts_[mobilizer_id];
  }
#endif

  /// Returns the current time in seconds.
  const T& get_time() const { return time_sec_; }

  void Print() {
    PRINT_VAR(q_.transpose());
    PRINT_VAR(v_.transpose());
    position_kinematics_.Print();
  }

 private:
  T time_sec_;
  VectorX<T> q_, v_;
  // Cached kinematics.
  mutable PositionKinematicsCache<T> position_kinematics_;

#if 0
  // Mobilizer contexts indexed by mobilizer id.
  std::vector<MobilizerContext<T>> mobilizer_contexts_;

  MobilizerContext<T> CreateMobilizerContext(
      const MultibodyTreeTopology& tree_topology, MobilizerIndex mobilizer_id)
  {
    const BodyNodeIndex body_node_id =
        tree_topology.mobilizers_[mobilizer_id].body_node;
    const BodyNodeTopology& node = tree_topology.body_nodes[body_node_id];
    const int rigid_positions_start = node.rigid_positions_start;
    const int num_rigid_positions = node.num_rigid_positions;
    const int rigid_velocities_start = node.rigid_velocities_start;
    const int num_rigid_velocities = node.num_rigid_velocities;

    MobilizerPositionKinematics<T> position_kinematics = 
        CreateMobilizerPositionKinematics(tree_topology, mobilizer_id);

    // Pointer to its local positions entry.
    T* rigid_positions = q_.data() + rigid_positions_start;
    // Pointer to its local velocities entry.
    T* rigid_velocities = v_.data() + rigid_velocities_start;

    MobilizerContext<T> mobilizer_context(
        num_rigid_positions, num_rigid_velocities,
        rigid_positions, rigid_velocities,
        position_kinematics);
    
    return mobilizer_context;
  }

  MobilizerPositionKinematics<T> CreateMobilizerPositionKinematics(
      const MultibodyTreeTopology& tree_topology, MobilizerIndex mobilizer_id)
  {
    const BodyNodeIndex body_node_id =
        tree_topology.mobilizers_[mobilizer_id].body_node;
    const BodyNodeTopology& node = tree_topology.body_nodes[body_node_id];
    //const int rigid_positions_start = node.rigid_positions_start;
    //const int num_rigid_positions = node.num_rigid_positions;
    const int rigid_velocities_start = node.rigid_velocities_start;
    //const int num_rigid_velocities = node.num_rigid_velocities;

    Isometry3<T>* X_FM =
        &(position_kinematics_.get_mutable_X_FM_pool()[body_node_id]);
    T* H_FM = 
        position_kinematics_.
            get_mutable_H_FM_pool()[rigid_velocities_start].mutable_data();

    return MobilizerPositionKinematics<T>(X_FM, H_FM);
  }
#endif
};

}  // namespace multibody
}  // namespace drake
