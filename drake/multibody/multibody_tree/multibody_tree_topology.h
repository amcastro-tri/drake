#pragma once

#include "drake/multibody/multibody_tree/multibody_indexes.h"

#include <algorithm>
#include <vector>

namespace drake {
namespace multibody {

struct MultibodyTreeTopology;

struct BodyTopology {
  BodyIndex id{BodyIndex::Invalid()};
  int level{-1};
  BodyIndex parent_body{BodyIndex::Invalid()};
  std::vector<BodyIndex> child_bodies;

  /// Returns `true` if @p body represents a child body entry in this body
  /// topology.
  /// Complexity: up to linear in the number of children.
  bool has_child(BodyIndex body) {
    return std::find(
        child_bodies.begin(), child_bodies.end(), body) != child_bodies.end();
  }
};

struct JointTopology {
  BodyIndex inboard_body{JointIndex::Invalid()};
  BodyIndex outboard_body{JointIndex::Invalid()};
};

struct MultibodyTreeTopology {
  std::vector<BodyTopology> bodies_;
  std::vector<JointTopology> joints_;
  int num_levels;
};

}  // namespace multibody
}  // namespace drake
