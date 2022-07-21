#pragma once

#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
struct CouplerConstraintSpecs {
  // First joint with position q₀.
  JointIndex joint0_index;
  // Second joint with position q₁.
  JointIndex joint1_index;
  // Gear ratio g defined such that q₀ = g⋅q₁.
  T gear_ratio{1.0};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
