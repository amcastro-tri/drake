#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

/// For internal use only of the MultibodyTree implementation.
/// While all code that is common to any node can be placed in the BodyNode
/// class, %BodyNodeImpl provides compile-time fixed-size BodyNode
/// implementations so that all operations can be perfomed with fixed-size
/// stack-allocated Eigen variables.
/// In particular, most of the across mobilizer code for velocity kinematics
/// lives in this class since the acrooss mobilizer Jacobian matrices have a
/// compile-time fixed size.
template <typename T, int  num_positions, int num_velocities>
class BodyNodeImpl : public BodyNode<T> {
 public:
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = num_positions, nv = num_velocities};

  /// A node is a computational cell encompassing a Body in a MultibodyTree
  /// and the inboard Mobilizer that connects this body to the tree. Given a
  /// body and its inboard mobilizer in a MultibodyTree this constructor
  /// creates the corresponding %BodyNode. See the BodyNode class documentation
  /// for details on how a BodyNode is defined.
  /// @param[in] body The body B associated with `this` node. It must be a valid
  ///                 pointer.
  /// @param[in] mobilizer The mobilizer associated with this `node`. It can
  ///                      only be a `nullptr` for the **world** body.
  BodyNodeImpl(const Body<T>* body, const Mobilizer<T>* mobilizer) :
      BodyNode<T>(body, mobilizer) {}

  // TODO(amcastro-tri): Implement methods for computing velocity kinematics
  // using fixed-size Eigen matrices.
};

}  // namespace multibody
}  // namespace drake
