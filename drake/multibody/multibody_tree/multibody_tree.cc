#include "multibody_tree.h"

#include "drake/common/eigen_types.h"

#include <queue>

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace multibody {

using Eigen::Vector3d;

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // The "world" body has infinite mass.
  Body<double>::CreateBody(this, MassProperties<double>::InfiniteMass());
}

template <typename T>
Body<T>* MultibodyTree<T>::AddBody(std::unique_ptr<Body<T>> body) {
  DRAKE_DEMAND(body != nullptr);
  InvalidateTopology();
  Body<T>* body_ptr = body.get();
  // Access private body members through an attorney-client idiom.
  Body<T>::TopologyAccess::set_parent_tree(body_ptr, this);
  Body<T>::TopologyAccess::set_id(body_ptr, BodyIndex(get_num_bodies()));
  bodies_.push_back(std::move(body));
  return body_ptr;
}

template <typename T>
void MultibodyTree<T>::CompileTopology() {
  if (topology_.is_valid) {
    throw std::runtime_error(
        "Attempting to compile and already compiled topology");
  }

  auto& body_topologies = topology_.bodies_;

  // Resize body topologies, assign id.
  body_topologies.resize(get_num_bodies());
  for (BodyIndex ibody(0); ibody < get_num_bodies(); ++ibody) {
    body_topologies[ibody].id = ibody;
  }

  // Create parent/child connections.
  // Skip JointIndex(0), the "world's joint".
  for (JointIndex ijoint(0); ijoint < get_num_joints(); ++ijoint) {
    const Joint<T>& joint = get_joint(ijoint);
    JointTopology joint_topology = joint.get_topology();
    BodyIndex inbody = joint_topology.inboard_body;
    BodyIndex outbody = joint_topology.outboard_body;

    if (body_topologies[outbody].parent_body.is_valid()) {
      // TODO: come up with a more verbose error message.
      throw std::runtime_error("Attempting to assign parent body twice");
    }
    body_topologies[outbody].parent_body = inbody;

    if (body_topologies[inbody].has_child(outbody)) {
      // TODO: come up with a more verbose error message.
      throw std::runtime_error("Attempting to add outboard body twice");
    }
    body_topologies[inbody].child_bodies.push_back(outbody);
  }

  // Compute body levels in the tree. Root is the zero level.
  // Breadth First Traversal (Level Order Traversal).
  std::queue<BodyIndex> queue;
  queue.push(BodyIndex(0));  // Starts at the root.
  int num_levels = 1;  // At least one level with the world body at the root.
  while (!queue.empty()) {
    BodyIndex current = queue.front();

    // Computes level.
    int level = 0;  // level = 0 for the world body.
    if (current != 0) {  // Not the world body.
      BodyIndex parent = body_topologies[current].parent_body;
      level = body_topologies[parent].level + 1;
    }
    body_topologies[current].level = level;
    num_levels = std::max(num_levels, level + 1);

    // Pushes children to the back of the queue and pops current.
    for (BodyIndex child: body_topologies[current].child_bodies) {
      queue.push(child);  // Pushes at the back.
    }
    queue.pop();  // Pops front element.
  }
  topology_.num_levels = num_levels;

  // Mark this tree's topology as valid.
  topology_.validate();
}

template <typename T>
void MultibodyTree<T>::Compile() {
  // Fill in MultibodyTree<T>::body_levels_
  // It is assumed there is one joint per body.
  DRAKE_ASSERT(get_num_bodies() == get_num_joints());
  // Compute MultibodyTree<T>::topology_
  CompileTopology();

  auto& body_topologies = topology_.bodies_;

  // Updates each body local copy of the topology (only this method is granted
  // access to body's internals).
  for (BodyIndex ibody(0); ibody < get_num_bodies(); ++ibody) {
    // Access private body members through an attorney-client idiom.
    Body<T>::TopologyAccess::set_topology(
        bodies_[ibody].get(), body_topologies[ibody]);
  }

  // Now that levels are computed by CompileTopology(), create a list of bodies
  // ordered by level for fast traversals.
  body_levels_.resize(topology_.num_levels);
  for (BodyIndex ibody(0); ibody < get_num_bodies(); ++ibody) {
    const int level = body_topologies[ibody].level;
    body_levels_[level].push_back(ibody);
  }
}

template <typename T>
void MultibodyTree<T>::PrintTopology() const {
  auto& body_topologies = topology_.bodies_;

  for (BodyIndex ibody(0); ibody < get_num_bodies(); ++ibody) {
    PRINT_VAR(ibody);
    PRINT_VAR(body_topologies[ibody].level);
    PRINT_VAR(body_topologies[ibody].parent_body);
    for (BodyIndex child: body_topologies[ibody].child_bodies) {
      PRINT_VAR(body_topologies[child].id);
    }
  }

  PRINT_VAR("");

  PRINT_VAR(topology_.num_levels);
  for (int level = 0; level < topology_.num_levels; ++level) {
    PRINT_VAR(level);
    for (BodyIndex ibody: body_levels_[level]) {
      PRINT_VAR(ibody);
    }
  }
}

template <typename T>
const Body<T>& MultibodyTree<T>::get_body(BodyIndex body_id) const {
  DRAKE_ASSERT(body_id.is_valid() && body_id < get_num_bodies());
  return *bodies_[body_id];
}

template <typename T>
Body<T>& MultibodyTree<T>::get_mutable_body(BodyIndex body_id) const {
  DRAKE_ASSERT(body_id.is_valid() && body_id < get_num_bodies());
  return *bodies_[body_id];
}


template <typename T>
const Body<T>& MultibodyTree<T>::get_body_inboard_body(
    BodyIndex body_id) const {
  return get_body(get_body(body_id).get_topology().parent_body);
}

template <typename T>
const Joint<T>& MultibodyTree<T>::get_joint(JointIndex joint_id) const {
  DRAKE_ASSERT(joint_id.is_valid() && joint_id < get_num_joints());
  return *joints_[joint_id];
}

template <typename T>
void MultibodyTree<T>::UpdatePositionKinematics(PositionKinematicsCache<T>* pc) const {

}

template class MultibodyTree<double>;

}  // namespace multibody
}  // namespace drake
