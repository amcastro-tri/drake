#pragma once

#include "drake/common/drake_assert.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"

#include <algorithm>
#include <vector>

namespace drake {
namespace multibody {

struct BodyTopology {
  BodyIndex id{BodyIndex::Invalid()};  // Unique id in the MultibodyTree.
  int level{-1};  // Depth level in the MultibodyTree, level = 0 for the world.

  // Connection to inboard/outboard bodies.
  BodyIndex parent_body{BodyIndex::Invalid()};
  std::vector<BodyIndex> child_bodies;

  // Connection to inboard/outboard joints.
  MobilizerIndex inboard_mobilizer{MobilizerIndex::Invalid()};
  std::vector<MobilizerIndex> outboard_mobilizer;

  std::vector<FrameIndex> material_frames;

  // The index of the BodyNode associated with this body in the MultibodyTree.
  BodyNodeIndex body_node;

  int get_num_children() const { return static_cast<int>(child_bodies.size());}

  int get_num_material_frames() const
  {
    return static_cast<int>(material_frames.size());
  }

  /// Returns `true` if @p body represents a child body entry in this body
  /// topology.
  /// Complexity: up to linear in the number of children.
  bool has_child(BodyIndex body) {
    return std::find(
        child_bodies.begin(), child_bodies.end(), body) != child_bodies.end();
  }
};

struct MobilizerTopology {
  MobilizerIndex id{MobilizerIndex::Invalid()};
  FrameIndex inboard_frame{FrameIndex::Invalid()};
  FrameIndex outboard_frame{FrameIndex::Invalid()};
  BodyIndex inboard_body{MobilizerIndex::Invalid()};
  BodyIndex outboard_body{MobilizerIndex::Invalid()};
  // The index of the BodyNode associated with this body in the MultibodyTree.
  BodyNodeIndex body_node;

  // MobilizerIndexingInfo
  int num_positions;
  int positions_start;
  int num_velocities;
  int velocities_start;

  void SetIndexingInfo(int position_start, int velocity_start,
                       int number_positions, int number_velocities) {
    num_positions = number_positions;
    positions_start = position_start;
    num_velocities = number_velocities;
    velocities_start = velocity_start;
  }
};

struct BodyNodeTopology {
  BodyNodeIndex id{BodyNodeIndex::Invalid()};  // Unique id in the parent tree.
  int level{-1};  // Depth level in the MultibodyTree, level = 0 for the world.

  BodyIndex body;
  MobilizerIndex mobilizer;

  // The unique identifier to the parent BodyNode of this node.
  BodyNodeIndex parent_body_node;

  int num_rigid_positions;
  int rigid_positions_start;
  int num_rigid_velocities;
  int rigid_velocities_start;

  int num_flexible_positions;
  int flexible_positions_start;
  int num_flexible_velocities;
  int flexible_velocities_start;

  // Index into the pool of material frames attached to a body (X_BF_pool) that
  // references the pose of the inboard frame F measured and expressed in the
  // parent body frame P.
  int X_PF_index{-1};

  // The "fixed" frame F is the parent body frame P.
  bool F_equals_P{false};

  void SetArrayIndexes(int position_start, int velocity_start,
                       int number_rigid_positions, int number_flexible_positions,
                       int number_rigid_velocities, int number_flexible_velocities) {
    // Positions are arranged first: Rigid DOF's followed by flexible DOF's.
    rigid_positions_start = position_start;
    num_rigid_positions = number_rigid_positions;
    flexible_positions_start = position_start + number_rigid_positions;
    num_flexible_positions = number_flexible_positions;

    // Velocities follow positions: Rigid DOF's followed by flexible DOF's.
    rigid_velocities_start = velocity_start;
    num_rigid_velocities = number_rigid_velocities;
    flexible_velocities_start = velocity_start + number_rigid_velocities;
    num_flexible_velocities = number_flexible_velocities;
  }
};

struct MaterialFrameTopology {
  // Unique identifier in the MultibodyTree.
  FrameIndex id{FrameIndex::Invalid()};
  // Unique identifier of the body this material frame attaches to.
  BodyIndex body_id{BodyIndex::Invalid()};
  // The index of the BodyNode associated with body_id.
  BodyNodeIndex body_node{BodyNodeIndex::Invalid()};
  // Local frame id or index in the body.
  int local_id{-1};
  // Index in the pool of frame poses in the position kinematics cache.
  int X_BF_index{-1};

  /// `true` if this frame corresponds to a BodyFrame. `false` otherwise.
  bool is_body_frame() const { return local_id == 0;}
};

struct MultibodyTreeTopology {
  std::vector<BodyTopology> bodies_;
  std::vector<MobilizerTopology> mobilizers_;
  std::vector<BodyNodeTopology> body_nodes;
  std::vector<MaterialFrameTopology> material_frames;
  int num_levels{0};
  int num_rigid_positions{0}, num_rigid_velocities{0};
  int num_flexible_positions{0}, num_flexible_velocities{0};
  int num_positions{0}, num_velocities{0};
  int X_BF_pool_size{0};

  int get_num_bodies() const {return static_cast<int>(bodies_.size()); }
  int get_num_mobilizers() const {return static_cast<int>(mobilizers_.size()); }
  int get_num_body_nodes() const {return static_cast<int>(body_nodes.size()); }
  int get_num_material_frames() const
  {
    return static_cast<int>(material_frames.size());
  }

  BodyIndex add_body() {
    invalidate();
    BodyTopology body;
    body.id = BodyIndex(get_num_bodies());
    bodies_.push_back(body);
    return body.id;
  }

  FrameIndex add_material_frame(const MaterialFrameTopology& frame) {
    const BodyIndex body_id = frame.body_id;
    DRAKE_ASSERT(is_valid_body_id(body_id));
    invalidate();
    FrameIndex frame_id(get_num_material_frames());
    BodyTopology& body_topology = bodies_[body_id];
    int local_id = body_topology.get_num_material_frames();
    material_frames.push_back(frame);
    material_frames.back().id = frame_id;
    material_frames.back().local_id = local_id;
    body_topology.material_frames.push_back(frame_id);
    return frame_id;
  }

  /// Adds a MobilizerTopology to the MultibodyTreeTopology.
  /// @p mobilizer is expected to only have valid inboard/outbard frames and
  /// bodies initialized.
  MobilizerIndex add_mobilizer(const MobilizerTopology& mobilizer) {
    const FrameIndex inboard_frame = mobilizer.inboard_frame;
    const FrameIndex outboard_frame = mobilizer.outboard_frame;
    DRAKE_ASSERT(is_valid_frame_id(inboard_frame));
    DRAKE_ASSERT(is_valid_frame_id(outboard_frame));

    BodyIndex inboard_body = material_frames[inboard_frame].body_id;
    DRAKE_ASSERT(is_valid_body_id(inboard_body));
    DRAKE_ASSERT(mobilizer.inboard_body == inboard_body);

    BodyIndex outboard_body = material_frames[outboard_frame].body_id;
    DRAKE_ASSERT(is_valid_body_id(outboard_body));
    DRAKE_ASSERT(mobilizer.outboard_body == outboard_body);

    MobilizerIndex id(get_num_mobilizers());
    mobilizers_.push_back(mobilizer);
    mobilizers_.back().id = id;
    invalidate();
    return mobilizer.id;
  }

  bool is_valid{false};
  /// Topology is invalidated when bodies or joints are added to the tree.
  /// It gets validated by MultibodyTree::Compile().
  void invalidate() { is_valid = false; }
  void validate() { is_valid = true; }

  bool is_valid_body_id(BodyIndex id) {
    return id.is_valid() && id < get_num_bodies();
  }

  bool is_valid_frame_id(FrameIndex id) {
    return id.is_valid() && id < get_num_material_frames();
  }
};

}  // namespace multibody
}  // namespace drake
