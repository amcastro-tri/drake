#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/joint.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyTree {
 public:

  /// Creates a MultibodyTree containing only a *world* body (with unique BodyId
  /// equal to 0).
  MultibodyTree();

  /// Takes ownership of @p body and assigns a unique id to it.
  /// @note
  /// This method is called from within CreateBody(). It is not meant to be
  /// called by end users.
  Body<T>* AddBody(std::unique_ptr<Body<T>> body);

  /// Takes ownership of @p joint and assigns a unique id to it.
  /// @note
  /// This method is called from within the CreateJointType method.
  /// It is not meant to be called by end users.
  template <class JointType>
  JointType* AddJoint(std::unique_ptr<JointType> joint) {
    DRAKE_DEMAND(joint != nullptr);
    // Call similar to invalidateSubsystemTopologyCache() in Simbody.
    // Notify that the topology of the MultibodyTree changed.
    InvalidateTopology();
    //body->set_id(get_num_bodies());
    JointType* joint_ptr = joint.get();
    joint_ptr->set_parent_tree(this);
    joints_.push_back(std::move(joint));
    return joint_ptr;
  }

  /// Returns the number of bodies in the MultibodyTree including including the
  /// *world* body. Therefore the minimum number of bodies in a MultibodyTree to
  /// which no other bodies have been added, is one.
  int get_num_bodies() const { return static_cast<int>(bodies_.size()); }

  int get_num_joints() const { return static_cast<int>(joints_.size()); }

  /// Returns a constant reference to the *world* body.
  const Body<T>& get_world_body() const { return *bodies_[0]; }

#if 0
  /// Takes ownership of @p joint and assigns a unique id to it.
  /// @note
  /// This method is called from within CreateBody(). It is not meant to be
  /// called by end users.
  template <typename JointType>
  JointType* AddJoint(std::unique_ptr<JointType> joint) {
    DRAKE_DEMAND(body != nullptr);
    InvalidateTopology();
    joint->set_id(get_num_joints());
    JointType* joint_ptr = joint.get();
    joints_.push_back(std::move(joint));
    return joint_ptr;
  }
#endif

 private:
  // Invalidates tree topology when it changes.
  // See methods AddBody() and AddJoint().
  void InvalidateTopology() {
    //topology_.invalidate();
  }

  std::vector<std::unique_ptr<Body<T>>> bodies_;
  std::vector<std::unique_ptr<Joint<T>>> joints_;
  //std::vector<std::unique_ptr<Joint>> joints_;
  // Notice this member is not templated on <T>.
  // It'd be nice to be able to simple copy topology_ on CloneTo<ScalarTo>()
  // calls. That could be accomplished if topology_ is desribed in terms of
  // indexes only.
  //MultibodyTreeTopology topology_;
};

}  // namespace multibody
}  // namespace drake
