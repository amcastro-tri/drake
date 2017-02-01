#pragma once

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

  // The index of the BodyNode associated with this body in the MultibodyTree.
  BodyNodeIndex body_node;

  /// Returns `true` if @p body represents a child body entry in this body
  /// topology.
  /// Complexity: up to linear in the number of children.
  bool has_child(BodyIndex body) {
    return std::find(
        child_bodies.begin(), child_bodies.end(), body) != child_bodies.end();
  }
};

struct MobilizerTopology {
  BodyIndex inboard_body{MobilizerIndex::Invalid()};
  BodyIndex outboard_body{MobilizerIndex::Invalid()};
  // The index of the BodyNode associated with this body in the MultibodyTree.
  BodyNodeIndex body_node;
};

struct BodyNodeTopology {
  BodyNodeIndex id{BodyNodeIndex::Invalid()};  // Unique id in the parent tree.
  int level{-1};  // Depth level in the MultibodyTree, level = 0 for the world.

  BodyIndex body;
  MobilizerIndex mobilizer;

  int num_rigid_positions;
  int rigid_positions_start;
  int num_rigid_velocities;
  int rigid_velocities_start;

  int num_flexible_positions;
  int flexible_positions_start;
  int num_flexible_velocities;
  int flexible_velocities_start;

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

struct MultibodyTreeTopology {
  std::vector<BodyTopology> bodies_;
  std::vector<MobilizerTopology> mobilizers_;
  std::vector<BodyNodeTopology> body_nodes;
  int num_levels{0};
  int num_rigid_positions{0}, num_rigid_velocities{0};
  int num_flexible_positions{0}, num_flexible_velocities{0};
  int num_positions{0}, num_velocities{0};

  int get_num_bodies() const {return static_cast<int>(bodies_.size()); }
  int get_num_mobilizers() const {return static_cast<int>(mobilizers_.size()); }
  int get_num_body_nodes() const {return static_cast<int>(body_nodes.size()); }

  bool is_valid{false};
  /// Topology is invalidated when bodies or joints are added to the tree.
  /// It gets validated by MultibodyTree::Compile().
  void invalidate() { is_valid = false; }
  void validate() { is_valid = true; }
};

}  // namespace multibody
}  // namespace drake
