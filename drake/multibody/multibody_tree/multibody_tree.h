#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/joint.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"
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

  const Body<T>& get_body(BodyIndex body_id) const;

  Body<T>& get_mutable_body(BodyIndex body_id) const;

  const Body<T>& get_body_inboard_body(BodyIndex body_id) const;

  /// This method must be called after all elements in the tree (joints, bodies,
  /// force elements, constraints) were added and before any computations.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and any other elements connect with each other.
  /// Sizes needed to allocate Context and Cache entries are determined with
  /// this call.
  void Compile();

  /// Creates a default context allocated with AllocateContext() and initialized
  /// by SetDefaults().
  //std::unique_ptr<Context<T>> CreateDefaultContext() const {
  //  std::unique_ptr<Context<T>> context = AllocateContext();
  //  SetDefaults(context.get());
  //  return context;
 // }

  void UpdatePositionKinematics(PositionKinematicsCache<T>* pc) const;

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

  void CompileTopology();

  std::vector<std::unique_ptr<Body<T>>> bodies_;
  std::vector<std::unique_ptr<Joint<T>>> joints_;
  //std::vector<std::unique_ptr<Joint>> joints_;
  // Notice this member is not templated on <T>.
  // It'd be nice to be able to simple copy topology_ on CloneTo<ScalarTo>()
  // calls. That could be accomplished if topology_ is desribed in terms of
  // indexes only.
  //MultibodyTreeTopology topology_;

  // Topology cache: This is all the information needed regarding the
  // connectivity of the system that allows to perform efficient traversals for
  // multibody algorithms.

  // for the level-th level in the tree, body_levels_[level] contains the idexes
  // of all bodies in that level. level = 0 refers to the world body.
  std::vector<std::vector<BodyIndex>> body_levels_;

};

}  // namespace multibody
}  // namespace drake
