#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <typename T>
class BodyNode : public MultibodyTreeElement<BodyNode<T>, BodyNodeIndex> {
 public:
  BodyNode(BodyIndex body_id, MobilizerIndex mobilizer_id) :
      body_id_(body_id), mobilizer_id_(mobilizer_id) {}

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

  int get_rigid_positions_start() const { return rigid_positions_start_;}
  int get_num_rigid_positions() const { return num_rigid_positions_;}
  int get_rigid_velocities_start() const { return rigid_velocities_start_;}

  const Body<T>& get_body() const {
    return this->get_parent_tree().get_body(body_id_);
  }

  const Mobilizer<T>& get_mobilizer() const {
    return this->get_parent_tree().get_mobilizer(mobilizer_id_);
  }

  MobilizerIndex get_mobilizer_id() const { return mobilizer_id_; }

  BodyIndex get_body_id() const { return body_id_; }

  /// Computes the rigid body inertia matrix for a given, fixed, value of the
  /// flexible generalized coordinates @p qf.
  //virtual SpatialMatrix DoCalcSpatialInertia(const VectorX<T>& qf) const = 0;

  void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const {
    const Mobilizer<T>& mobilizer = get_mobilizer();

    // Update position kinematics that depend on mobilizers only:
    // - X_FM(q), H_FM(q), HdotTimesV_FM(q)
    mobilizer.UpdatePositionKinematics(context);

    //Similarly here for body updates ....
    // Then right here we can do across body updates assuming we are in a
    // tip-to-base loop...
  }

 private:
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
