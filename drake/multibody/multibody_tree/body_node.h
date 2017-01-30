#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <typename T>
class BodyNode {
 public:
  BodyNode(BodyIndex body_id, MobilizerIndex mobilizer_id) :
      body_id_(body_id), mobilizer_id_(mobilizer_id) {}

  BodyNode(const MultibodyTree<T>* parent_tree) : parent_tree_(parent_tree) {}

  void SetArrayIndexes(int position_start, int velocity_start,
                       int num_rigid_positions, int num_flexible_positions,
                       int num_rigid_velocities, int num_flexible_velocities) {
    // Positions are arranged first: Rigid DOF's followed by flexible DOF's.
    rigid_positions_start_ = position_start;
    num_rigid_positions_ = num_rigid_positions;
    flexible_positions_start_ = position_start + num_rigid_positions;
    num_flexible_positions_ = num_flexible_positions;

    // Velocities follow positions: Rigid DOF's followed by flexible DOF's.
    rigid_velocities_start_ = velocity_start;
    num_rigid_velocities_ = num_rigid_velocities;     
    flexible_velocities_start_ = velocity_start + num_rigid_velocities;
    num_flexible_velocities_ = num_flexible_velocities;
  }

  /// Computes the rigid body inertia matrix for a given, fixed, value of the
  /// flexible generalized coordinates @p qf.
  //virtual SpatialMatrix DoCalcSpatialInertia(const VectorX<T>& qf) const = 0;

  void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const {
    PositionKinematicsCache<T>* pc =
        context.get_mutable_position_kinematics();
    const VectorX<T>& tree_positions = context.get_positions();

    const Mobilizer<T>& mobilizer = get_mobilizer();
    // positions local to this node.
    const auto q = get_rigid_positions_slice(tree_positions);
    //const auto q = tree_positions.template segment(position_start_, num_positions_);

    // Notice here how I ask a simple question (no context or anything weired
    // but just vector arguments) to the Mobilizer
    get_mutable_X_FM(pc) = mobilizer.CalcX_FM(q);
    //get_mutable_H_FM(pc) = mobilizer.CalcH_FM(q);

    //Similarly here for body updates ....

    // Then right here we can do across body updates assuming we are in a
    // tip-to-base loop...
  }

  const Body<T>& get_body(BodyIndex body_id) const {
    return parent_tree_->get_body(body_id);
  }

  const Mobilizer<T>& get_mobilizer(MobilizerIndex mobilizer_id) const {
    return parent_tree_->get_mobilizer(mobilizer_id);
  }

 private:
  // Helper methods to extract local variables from the tree global variables.

  // Figure out here what "auto" is.
  const auto get_rigid_positions_slice(
      const Eigen::Ref<const VectorX<T>>& tree_positions) {
    return tree_positions.template segment(
        rigid_positions_start_, num_rigid_positions_);
  }

  Isometry3<T>& get_mutable_X_FM(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_FMs()[mobilizer_id_];
  }

  MultibodyTree<T>* parent_tree_{nullptr};
  BodyIndex body_id_;
  MobilizerIndex mobilizer_id_;

  // Position indexes
  int rigid_positions_start_{-1};
  int num_rigid_positions_{0};
  int flexible_positions_start_{-1};
  int num_flexible_positions_{0};

  // Velocity indexes.
  int rigid_velocities_start_{-1};
  int num_rigid_velocities_{0};
  int flexible_velocities_start_{-1};
  int num_flexible_velocities_{0};
};

}  // namespace multibody
}  // namespace drake
