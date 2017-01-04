#pragma once

#include "drake/multibody/multibody_tree/multibody_indexes.h"

#include <vector>

namespace drake {
namespace multibody {

struct MultibodyTreeTopology;

struct BodyTopology {
  int level_;
  BodyIndex parent_body_;
  std::vector<BodyIndex> child_bodies_;
};

struct JointTopology {
  BodyIndex inboard_body_;
  BodyIndex outboard_body_;
};

struct MultibodyTreeTopology {
  std::vector<BodyTopology> bodies_;
  std::vector<JointTopology> joints_;
};

}  // namespace multibody
}  // namespace drake
