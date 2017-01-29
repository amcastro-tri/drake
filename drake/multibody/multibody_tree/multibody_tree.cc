#include "drake/multibody/multibody_tree/multibody_tree.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"

#include <queue>

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace multibody {

using Eigen::Vector3d;

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // The "world" body has infinite mass.
  AddBody(std::make_unique<Body<T>>(MassProperties<double>::InfiniteMass()));
}

template <typename T>
Body<T>* MultibodyTree<T>::AddBody(std::unique_ptr<Body<T>> body) {
  DRAKE_DEMAND(body != nullptr);
  InvalidateTopology();
  Body<T>* body_raw_ptr = body.get();
  body_raw_ptr->set_parent_tree(this);
  body_raw_ptr->set_id(BodyIndex(get_num_bodies()));
  bodies_.push_back(std::move(body));
  return body_raw_ptr;
}

#if 0
template <typename T>
void MultibodyTree<T>::CompileTopology() {
  if (topology_.is_valid) return;  // Nothing new to compile.

  auto& body_topologies = topology_.bodies_;

  // Resize body topologies, assign id.
  body_topologies.resize(get_num_bodies());
  for (BodyIndex ibody(0); ibody < get_num_bodies(); ++ibody) {
    body_topologies[ibody].id = ibody;
  }

  // Create parent/child connections.
  // Skip MobilizerIndex(0), the "world's joint".
  for (MobilizerIndex ijoint(0); ijoint < get_num_joints(); ++ijoint) {
    const Joint<T>& joint = get_joint(ijoint);
    JointTopology joint_topology = joint.get_topology();
    BodyIndex inbody = joint_topology.inboard_body;
    BodyIndex outbody = joint_topology.outboard_body;

    if (body_topologies[outbody].parent_body.is_valid()) {
      // TODO: come up with a more verbose error message.
      throw std::runtime_error("Attempting to assign parent body twice");
    }
    body_topologies[outbody].parent_body = inbody;
    body_topologies[outbody].inboard_mobilizer = ijoint;

    if (body_topologies[inbody].has_child(outbody)) {
      // TODO: come up with a more verbose error message.
      throw std::runtime_error("Attempting to add outboard body twice");
    }
    body_topologies[inbody].child_bodies.push_back(outbody);
    body_topologies[inbody].outboard_mobilizer.push_back(ijoint);
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

  // Compute sizes and offsets.
  // Arrange variables in a from base to tip order.
  // Skip the world.
  int position_start = 0, velocity_start = 0;
  for (int level = 1; level < get_num_levels(); ++level) {
    for (BodyIndex ibody: body_levels_[level]) {
      MobilizerIndex ijoint = topology_.bodies_[ibody].inboard_mobilizer;
      joints_[ijoint]->set_start_indexes(position_start, velocity_start);
      position_start += joints_[ijoint]->get_num_qs();
      velocity_start += joints_[ijoint]->get_num_vs();
    }
  }
  num_positions_ = position_start;
  num_velocities_ = velocity_start;
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
#endif

template <typename T>
const Body<T>& MultibodyTree<T>::get_body(BodyIndex body_id) const {
  DRAKE_ASSERT(body_id.is_valid() && body_id < get_num_bodies());
  return *bodies_[body_id];
}

#if 0
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
const Joint<T>& MultibodyTree<T>::get_joint(MobilizerIndex joint_id) const {
  DRAKE_ASSERT(joint_id.is_valid() && joint_id < get_num_joints());
  return *joints_[joint_id];
}
#endif

#if 0
template <typename T>
void MultibodyTree<T>::UpdatePositionKinematics(
    const MultibodyTreeContext<T>& context) const {
  // Base-to-Tip recursion.
  // This skips the world, level = 0.
  for (int level = 1; level < get_num_levels(); ++level) {
    for (BodyNodeIndex body_node_index: body_node_levels_[level]) {
      BodyNode* node = body_nodes_[body_node_index].get();
      MobilizerIndex mobilizer_index = node->get_mobilizer_index();
      BodyIndex body_index = node->get_body_index();
      // Update position kinematics that depend on mobilizers only:
      // - X_FM, X_PB, X_WB, Ht_FM, Ht_PB_W
      body_nodes_[body_node_index]->UpdatePositionKinematicsCache(context);
      // Updates position kinematics quantities that depend on bodies:
      // - com_W, p_PB_W, M_Bo_W
      //bodies_[body_index]->UpdatePositionKinematicsCache(context);
    }
  }
}
#endif

template class MultibodyTree<double>;

}  // namespace multibody
}  // namespace drake
