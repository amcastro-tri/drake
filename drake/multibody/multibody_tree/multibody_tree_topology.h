#pragma once

#include "drake/multibody/multibody_tree/multibody_indexes.h"

#include <algorithm>
#include <vector>

namespace drake {
namespace multibody {

struct MultibodyTreeTopology;

struct BodyTopology {
  BodyIndex id{BodyIndex::Invalid()};  // Unique id in the MultibodyTree.
  int level{-1};  // Depth level in the MultibodyTree, level = 0 for the world.

  // Connection to inboard/outboard bodies.
  BodyIndex parent_body{BodyIndex::Invalid()};
  std::vector<BodyIndex> child_bodies;

  // Connection to inboard/outboard joints.
  MobilizerIndex inboard_mobilizer{MobilizerIndex::Invalid()};
  std::vector<MobilizerIndex> outboard_mobilizer;

  /// Returns `true` if @p body represents a child body entry in this body
  /// topology.
  /// Complexity: up to linear in the number of children.
  bool has_child(BodyIndex body) {
    return std::find(
        child_bodies.begin(), child_bodies.end(), body) != child_bodies.end();
  }
};

struct JointTopology {
  BodyIndex inboard_body{MobilizerIndex::Invalid()};
  BodyIndex outboard_body{MobilizerIndex::Invalid()};
};

struct MultibodyTreeTopology {
  std::vector<BodyTopology> bodies_;
  std::vector<JointTopology> joints_;
  int num_levels;
  bool is_valid{false};
  /// Topology is invalidated when bodies or joints are added to the tree.
  /// It gets validated by MultibodyTree::Compile().
  void invalidate() { is_valid = false; }
  void validate() { is_valid = true; }
};

struct MobilizerIndexesInfo {
  int position_start;
  int num_positions;
  int velocity_start;
  int num_velocities;
};

}  // namespace multibody
}  // namespace drake
