#include "drake/multibody/multibody_tree/multibody_tree.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mass_properties.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

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
  RigidBody<T>::Create(this, MassProperties<double>::InfiniteMass());
}

template <typename T>
BodyIndex MultibodyTree<T>::AddBody(std::unique_ptr<Body<T>> body) {
  DRAKE_DEMAND(body != nullptr);
  BodyIndex id = topology_.add_body();
  bodies_.push_back(std::move(body));
  return id;
}

template <typename T>
FrameIndex MultibodyTree<T>::AddMaterialFrame(std::unique_ptr<MaterialFrame<T>> frame) {
  DRAKE_DEMAND(frame != nullptr);
  FrameIndex id = topology_.add_material_frame(frame->get_topology());
  material_frames_.push_back(std::move(frame));
  return id;
}

template <typename T>
MobilizerIndex MultibodyTree<T>::AddMobilizer(
    std::unique_ptr<Mobilizer<T>> mobilizer) {
  DRAKE_DEMAND(mobilizer != nullptr);
  MobilizerIndex id = topology_.add_mobilizer(mobilizer->get_topology());
  mobilizers_.push_back(std::move(mobilizer));
  return id;
}

template <typename T>
void MultibodyTree<T>::CompileTopology() {
  if (topology_.is_valid) return;  // Nothing new to compile.

  auto& body_topologies = topology_.bodies_;
  auto& mobilizer_topologies = topology_.mobilizers_;
  auto& body_node_topologies = topology_.body_nodes;
  auto& frame_topologies = topology_.material_frames;

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

    // Sets mobilizer connectivities.
    mobilizer_topologies[mobilizer_id].inboard_body = inbody;
    mobilizer_topologies[mobilizer_id].outboard_body = outbody;

    // Sets body connectivities.
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

  // Check all bodies were reached. Otherwise throw an exception.
  // Bodies that were not reached were not assigned a valid level.
  for (BodyIndex body_id(0); body_id < get_num_bodies(); ++body_id) {
    if (body_topologies[body_id].level < 0) {
      throw std::runtime_error("Body was not assigned a mobilizer.");
    }
  }

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
  // BodyNode id's are not necessarily the same as Body id's.
  BodyNodeIndex body_node_id(0);
  int rigid_position_start = 0, rigid_velocity_start = 0;
  topology_.num_rigid_positions = 0;
  topology_.num_rigid_velocities = 0;
  topology_.num_flexible_positions = 0;
  topology_.num_flexible_velocities = 0;
  // Number of transformations X_BF to be allocated in the cache.
  topology_.X_BF_pool_size = 0;
  for (int level = 0; level < topology_.num_levels; ++level) {
    for (BodyIndex body_id: body_levels_[level]) {
      BodyTopology& body_topology = body_topologies[body_id];
      MobilizerIndex mobilizer_id = body_topology.inboard_mobilizer;

      // Record in BodyNodeTopology the Body and Mobilzer associated with it.
      body_node_topologies[body_node_id].id = body_node_id;
      body_node_topologies[body_node_id].level = level;
      body_node_topologies[body_node_id].body = body_id;
      body_node_topologies[body_node_id].mobilizer = mobilizer_id;
      body_node_topologies[body_node_id].child_bodies =
          body_topologies[body_id].child_bodies;

      // And vice-versa, store in BodyTopology and MobilizerTopology what
      // BodyNode they associate with.
      body_topologies[body_id].body_node = body_node_id;
      if (mobilizer_id.is_valid()) {// There is no inboard for the world.
        mobilizer_topologies[mobilizer_id].body_node = body_node_id;
      }

      // Compute sizes and offsets. Variables are arranged in a base to tip
      // order.
      // Sizes are zero for the world node.
      int num_rigid_positions = 0;
      int num_rigid_velocities = 0;
      int num_flexible_positions = 0;
      int num_flexible_velocities = 0;
      const Body<T> &body = get_body(body_id);
      if (body_id != kWorldBodyId) {
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
      if (mobilizer_id.is_valid()) {  // There is no inboard for the world.
        mobilizer_topologies[mobilizer_id].SetIndexingInfo(
            rigid_position_start, rigid_velocity_start,
            num_rigid_positions, num_rigid_velocities);
      }

      // Sets NodeTopology::X_PF_index, the index into the pool of material
      // frame poses for the F frame in the parent body P.
      if (mobilizer_id.is_valid()) {  // There is no mobilizer for the world.
        // Inboard frame F index.
        FrameIndex inboard_frame_id =
            mobilizer_topologies[mobilizer_id].inboard_frame;

        // The pool does not include the pose of the body frame.
        if (!topology_.material_frames[inboard_frame_id].is_body_frame()) {
          const int inboard_frame_local_id =
              topology_.material_frames[inboard_frame_id].local_id;
          const int X_BF_index = /* minus one because "0" is the body frame.*/
              topology_.X_BF_pool_size + inboard_frame_local_id - 1;
          frame_topologies[inboard_frame_id].X_BF_index = X_BF_index;

          // Same index for the parent fixed frame F in this BodyNode.
          body_node_topologies[body_node_id].F_equals_P = false;
          body_node_topologies[body_node_id].X_PF_index = X_BF_index;
        } else {
          // Indicates that frame F is the parent body frame P.
          body_node_topologies[body_node_id].F_equals_P = true;
        }

        // Inboard frame M index.
        FrameIndex outboard_frame_id =
            mobilizer_topologies[mobilizer_id].outboard_frame;
        // Each frame that is the outboard frame of a mobilizer gets associated
        // with the transform X_MB (the inverse of X_BM) for BodyNode
        // body_node_id.
        frame_topologies[outboard_frame_id].body_node = body_node_id;

        // The pool does not include the pose of the body frame.
        if (!topology_.material_frames[outboard_frame_id].is_body_frame()) {
          const int outboard_frame_local_id =
              topology_.material_frames[outboard_frame_id].local_id;
          const int X_BF_index = /* minus one because "0" is the body frame.*/
              topology_.X_BF_pool_size + outboard_frame_local_id - 1;
          frame_topologies[outboard_frame_id].X_BF_index = X_BF_index;
        }
      }  // if (mobilizer_id.is_valid())

      // Does not include the body frame in the pool of material frames.
      topology_.X_BF_pool_size += (body_topology.get_num_material_frames() - 1);

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

  // Now that each BodyTopology and BodyMobilizer was assigned a BodyNode, it
  // is possible to identify the parent BodyNode for each BodyNode.
  for (auto& body_node: body_node_topologies) {
    if (body_node.mobilizer.is_valid()) {  // i.e. has a parent.
      BodyIndex parent_body =
          mobilizer_topologies[body_node.mobilizer].inboard_body;
      body_node.parent_body_node = body_topologies[parent_body].body_node;
    }
  }

  // Updates bodies' topology.
  for (BodyIndex body_id(0);
       body_id < get_num_bodies(); ++body_id) {
    bodies_[body_id]->SetTopology(body_topologies[body_id]);
  }

  // Updates frames' topology.
  for (const auto& frame: material_frames_) {
    frame->SetTopology(topology_.material_frames[frame->get_id()]);
  }

  // Updates mobilizers' topology.
  for (MobilizerIndex mobilizer_id(0);
       mobilizer_id < get_num_mobilizers(); ++mobilizer_id) {
    mobilizers_[mobilizer_id]->SetTopology(mobilizer_topologies[mobilizer_id]);
  }

  // Mark this tree's topology as valid.
  topology_.validate();
}

template <typename T>
void MultibodyTree<T>::Compile() {
  // Computes MultibodyTree<T>::topology_
  CompileTopology();

  // Creates BodyNode's.
  auto& body_node_topologies = topology_.body_nodes;
  for (const auto& node_topology: body_node_topologies) {
    const Body<T>* body = bodies_[node_topology.body].get();
    const Mobilizer<T>* mobilizer = nullptr;
    // The world body node has not mobilizer.
    if (node_topology.mobilizer.is_valid())
      mobilizer = mobilizers_[node_topology.mobilizer].get();
    auto body_node = make_unique<BodyNode<T>>(
        node_topology, body, mobilizer);
    body_nodes_.push_back(std::move(body_node));
  }
}

template <typename T>
void MultibodyTree<T>::PrintTopology() const {

  std::cout << "Bodies: " << std::endl;
  for (const auto& body: bodies_) {
    body->PrintTopology();
  }
  std::cout << std::endl;

  std::cout << "Frames: " << std::endl;
  for (const auto& frame: material_frames_) {
    frame->PrintTopology();
  }
  std::cout << std::endl;

  std::cout << "Mobilizers: " << std::endl;
  for (const auto& mobilizer: mobilizers_) {
    mobilizer->PrintTopology();
  }
  std::cout << std::endl;

  std::cout << "Levels: " << std::endl;
  PRINT_VAR(topology_.num_levels);
  for (int level = 0; level < topology_.num_levels; ++level) {
    PRINT_VAR(level);
    for (BodyIndex ibody: body_levels_[level]) {
      PRINT_VAR(ibody);
    }
  }
  std::cout << std::endl;

  std::cout << "Body nodes: " << std::endl;
  for (const auto& body_node: body_nodes_)
    body_node->PrintTopology();

  PRINT_VAR(topology_.X_BF_pool_size);
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
void MultibodyTree<T>::UpdatePositionKinematicsCache(
    const MultibodyTreeContext<T>& context) const {

  // Loop over bodies to update their position dependent kinematics.
  // Essentially the pose X_BQ(qf_B) of each frame Q that is attached to the
  // body measured and expressed in the body frame B.
  // Notice this loop can be performed in any order and each X_BQ(qf_B) is
  // independent of all others. This could be performed even in parallel.
  // TODO(amcastro-tri): have a list of SoftBody's here since only their
  // kinematics changes with time.
  for (const auto& body: bodies_)
    body->UpdatePositionKinematicsCache(context);

  // Loop over all mobilizers to update their position dependent kinematics.
  // This updates the kinematics quantities only dependent on the rigid degrees
  // of freedom qr. These are: X_FM(qr), H_FM(qr), HdotTimesV(qr), N(qr).
  // Notice this loop can be performed in any order, even in parallel.
  for (const auto& mobilizer: mobilizers_)
    mobilizer->UpdatePositionKinematicsCache(context);

  // With the kinematics information across mobilizer's and the kinematics
  // information for each body, we are now in position to perform a base-to-tip
  // recursion to update world positions and parent to child body transforms.
  // This skips the world, level = 0.
  for (int level = 1; level < get_num_levels(); ++level) {
    for (BodyNodeIndex body_node_id: body_node_levels_[level]) {
      const BodyNode<T>& node = *body_nodes_[body_node_id];
      // Update per-node kinematics.
      node.UpdatePositionKinematicsCache(context);
    }
  }
  // TODO: Validate cache entry for PositionKinematicsCache.
}

template <typename T>
void MultibodyTree<T>::CalcCompositeBodyInertias(
    const MultibodyTreeContext<T>& context,
    eigen_aligned_std_vector<SpatialInertia<T>>& cbi_array) const {
  // TODO: check PositionKinematicsValidity.
  const PositionKinematicsCache<T>& pc = context.get_position_kinematics();

  // Perform a tip-to-base recursion. The world (level = 0) is not included.
  for (int level = get_num_levels() - 1; level > 0; --level) {
    for (BodyNodeIndex body_node_id: body_node_levels_[level]) {
      const auto& node = body_nodes_[body_node_id];
      // Update per-node kinematics.
      node->CalcCompositeBodyInertia_TipToBase(pc, cbi_array);
    }
  }
}

template class MultibodyTree<double>;

}  // namespace multibody
}  // namespace drake
