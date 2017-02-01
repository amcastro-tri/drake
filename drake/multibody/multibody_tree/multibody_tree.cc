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

using std::make_unique;

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

template <typename T>
void MultibodyTree<T>::CompileTopology() {
  if (topology_.is_valid) return;  // Nothing new to compile.

  auto& body_topologies = topology_.bodies_;
  auto& mobilizer_topologies = topology_.mobilizers_;

  mobilizer_topologies.resize(get_num_mobilizers());

  // Resize body topologies, assign id.
  body_topologies.resize(get_num_bodies());
  for (BodyIndex ibody(0); ibody < get_num_bodies(); ++ibody) {
    body_topologies[ibody].id = ibody;
  }

  // Create parent/child connections.
  for (MobilizerIndex mobilizer_id(0);
       mobilizer_id < get_num_mobilizers(); ++mobilizer_id) {
    const Mobilizer<T>& mobilizer = get_mobilizer(mobilizer_id);
    BodyIndex inbody = mobilizer.get_inboard_body_id();
    BodyIndex outbody = mobilizer.get_outboard_body_id();

    if (body_topologies[outbody].parent_body.is_valid()) {
      // TODO: come up with a more verbose error message.
      throw std::runtime_error("Attempting to assign parent body twice");
    }
    body_topologies[outbody].parent_body = inbody;
    body_topologies[outbody].inboard_mobilizer = mobilizer_id;

    if (body_topologies[inbody].has_child(outbody)) {
      // TODO: come up with a more verbose error message.
      throw std::runtime_error("Attempting to add outboard body twice");
    }
    body_topologies[inbody].child_bodies.push_back(outbody);
    body_topologies[inbody].outboard_mobilizer.push_back(mobilizer_id);
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

  // Now that levels are computed, create a list of bodies ordered by level
  // for fast traversals.
  body_levels_.resize(topology_.num_levels);
  for (BodyIndex body_id(0); body_id < get_num_bodies(); ++body_id) {
    const int level = body_topologies[body_id].level;
    body_levels_[level].push_back(body_id);
  }

  // Compile BodyNodeTopology
  topology_.body_nodes.resize(get_num_bodies());
  body_node_levels_.resize(topology_.num_levels);
  auto& body_node_topologies = topology_.body_nodes;
  // BodyNode id's are not necessarily the same as Body id's.
  BodyNodeIndex body_node_id(0);
  int rigid_position_start = 0, rigid_velocity_start = 0;
  topology_.num_rigid_positions = 0;
  topology_.num_rigid_velocities = 0;
  topology_.num_flexible_positions = 0;
  topology_.num_flexible_velocities = 0;
  for (int level = 0; level < topology_.num_levels; ++level) {
    for (BodyIndex body_id: body_levels_[level]) {
      MobilizerIndex mobilizer_id = body_topologies[body_id].inboard_mobilizer;

      // Record in BodyNodeTopology the Body and Mobilzer associated with it.
      body_node_topologies[body_node_id].id = body_node_id;
      body_node_topologies[body_node_id].body = body_id;
      body_node_topologies[body_node_id].mobilizer = mobilizer_id;

      // And vice-versa, store in BodyTopology and MobilizerTopology what
      // BodyNode they associate with.
      body_topologies[body_id].body_node = body_node_id;
      if (mobilizer_id.is_valid())  // There is no inboard for the world.
        mobilizer_topologies[mobilizer_id].body_node = body_node_id;

      // Compute sizes and offsets. Variables are arranged in a base to tip
      // order.
      // Sizes are zero for the world body.
      int num_rigid_positions = 0;
      int num_rigid_velocities = 0;
      int num_flexible_positions = 0;
      int num_flexible_velocities = 0;
      if (body_id != kWorldBodyId) {
        const Body<T> &body = get_body(body_id);
        const Mobilizer<T> &mobilizer = get_mobilizer(mobilizer_id);
        num_rigid_positions = mobilizer.get_num_positions();
        num_rigid_velocities = mobilizer.get_num_velocities();
        num_flexible_positions = body.get_num_positions();
        num_flexible_velocities = body.get_num_velocities();
      }

      body_node_topologies[body_node_id].SetArrayIndexes(
          rigid_position_start, rigid_velocity_start,
          num_rigid_positions, num_flexible_positions,
          num_rigid_velocities, num_flexible_velocities);

      rigid_position_start += (num_rigid_positions + num_flexible_positions);
      rigid_velocity_start += (num_rigid_velocities + num_flexible_velocities);

      topology_.num_rigid_positions += num_rigid_positions;
      topology_.num_rigid_velocities += num_rigid_velocities;

      topology_.num_flexible_positions += num_flexible_positions;
      topology_.num_flexible_velocities += num_flexible_velocities;

      body_node_levels_[level].push_back(body_node_id++);
    }
    topology_.num_positions =
        topology_.num_rigid_positions + topology_.num_flexible_positions;
    topology_.num_velocities =
        topology_.num_rigid_velocities + topology_.num_flexible_velocities;
  }

  // Mark this tree's topology as valid.
  topology_.validate();
}

template <typename T>
void MultibodyTree<T>::Compile() {
  // Computes MultibodyTree<T>::topology_
  CompileTopology();

  auto& body_topologies = topology_.bodies_;

  // TODO(amcastro-tri): Reuse information in topology_.body_nodes since all
  // sizes were already computed in CompileTopology().
  // Create a BodyNode<T> for each Body<T>, order by levels.
  // BodyNode id's are not necessarily the same as Body id's.
  BodyNodeIndex body_node_id(0);
  int position_start = 0, velocity_start = 0;
  for (int level = 0; level < get_num_levels(); ++level) {
    for (BodyIndex body_id: body_levels_[level]) {
      MobilizerIndex mobilizer_id = body_topologies[body_id].inboard_mobilizer;
      body_topologies[body_id].body_node = body_node_id;
      auto body_node = make_unique<BodyNode<T>>(body_id, mobilizer_id);
      body_node->set_parent_tree(this);
      body_node->set_id(body_node_id);

      // Compute sizes and offsets. Variables are arranged in a base to tip
      // order.
      // Sizes are zero for the world body.
      int num_rigid_positions = 0;
      int num_rigid_velocities = 0;
      int num_flexible_positions = 0;
      int num_flexible_velocities = 0;
      int H_FM_pool_start = 0;
      if (body_id != kWorldBodyId) {
        const Body<T> &body = get_body(body_id);
        const Mobilizer<T> &mobilizer = get_mobilizer(mobilizer_id);
        num_rigid_positions = mobilizer.get_num_positions();
        num_rigid_velocities = mobilizer.get_num_velocities();
        num_flexible_positions = body.get_num_positions();
        num_flexible_velocities = body.get_num_velocities();

        //mobilizer.SetArrayIndexes(position_start, velocity_start,
        //                          num_rigid_positions, num_rigid_velocities,
        //                          H_FM_pool_start);
        H_FM_pool_start += 6 * num_rigid_velocities;
      }

      body_node->SetArrayIndexes(position_start, velocity_start,
                                 num_rigid_positions, num_flexible_positions,
                                 num_rigid_velocities, num_flexible_velocities);

      position_start += (num_rigid_positions + num_flexible_positions);
      velocity_start += (num_rigid_velocities + num_flexible_velocities);
      // Finally, transfer ownership to the MultibodyTree.
      body_nodes_.push_back(std::move(body_node));
    }
  }

  // Create BodyNodeTopology entries.

}

template <typename T>
void MultibodyTree<T>::PrintTopology() const {
  auto& body_topologies = topology_.bodies_;

  for (BodyIndex body_id(0); body_id < get_num_bodies(); ++body_id) {
    PRINT_VAR(body_id);
    PRINT_VAR(body_topologies[body_id].level);
    PRINT_VAR(body_topologies[body_id].parent_body);
    for (BodyIndex child: body_topologies[body_id].child_bodies) {
      PRINT_VAR(body_topologies[child].id);
    }
    PRINT_VAR(body_topologies[body_id].body_node);
  }

  PRINT_VAR("");

  for (MobilizerIndex mobilizer_id(0);
       mobilizer_id < get_num_mobilizers(); ++mobilizer_id) {
    PRINT_VAR(mobilizer_id);
    PRINT_VAR(topology_.mobilizers_[mobilizer_id].inboard_body);
    PRINT_VAR(topology_.mobilizers_[mobilizer_id].outboard_body);
    PRINT_VAR(topology_.mobilizers_[mobilizer_id].body_node);
  }

  PRINT_VAR(topology_.num_levels);
  for (int level = 0; level < topology_.num_levels; ++level) {
    PRINT_VAR(level);
    for (BodyIndex ibody: body_levels_[level]) {
      PRINT_VAR(ibody);
    }
  }

  PRINT_VAR("");

  for (int level = 0; level < topology_.num_levels; ++level) {
    PRINT_VAR(level);
    for (BodyNodeIndex body_node_id: body_node_levels_[level]) {
      PRINT_VAR(body_node_id);
      PRINT_VAR(topology_.body_nodes[body_node_id].id);
      PRINT_VAR(topology_.body_nodes[body_node_id].body);
      PRINT_VAR(topology_.body_nodes[body_node_id].mobilizer);
      PRINT_VAR(topology_.body_nodes[body_node_id].num_rigid_positions);
      PRINT_VAR(topology_.body_nodes[body_node_id].num_rigid_velocities);
    }
  }

}

template <typename T>
const Body<T>& MultibodyTree<T>::get_body(BodyIndex body_id) const {
  DRAKE_ASSERT(body_id.is_valid() && body_id < get_num_bodies());
  return *bodies_[body_id];
}

template <typename T>
const Mobilizer<T>& MultibodyTree<T>::get_mobilizer(
    MobilizerIndex mobilizer_id) const {
  DRAKE_ASSERT(mobilizer_id.is_valid() && mobilizer_id < get_num_mobilizers());
  return *mobilizers_[mobilizer_id];
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

template <typename T>
void MultibodyTree<T>::UpdatePositionKinematics(
    const MultibodyTreeContext<T>& context) const {
  // Base-to-Tip recursion.
  // This skips the world, level = 0.
  for (int level = 1; level < get_num_levels(); ++level) {
    for (BodyNodeIndex body_node_id: body_node_levels_[level]) {
      const BodyNode<T>& node = *body_nodes_[body_node_id];

      // Update per-node kinematics.
      node.UpdatePositionKinematicsCache(context);

      // Updates position kinematics quantities that depend on bodies:
      // - com_W, p_PB_W, M_Bo_W
      //bodies_[body_index]->UpdatePositionKinematicsCache(context);
    }
  }
}

template class MultibodyTree<double>;

}  // namespace multibody
}  // namespace drake
