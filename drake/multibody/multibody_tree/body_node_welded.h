#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

namespace drake {
namespace multibody {

/// This class represents a BodyNode for nodes with zero degrees of freedom.
/// These include the world body and the WeldMobilzer.
/// This class not only encapsulate the right abstraction of a node with zero
/// degrees of freedom but also solves the problem of instantiating a
/// BodyNodeImpl with zero compile-time sizes, which leads to Eigen expressions
/// that assert at compile-time.
/// Also, this class results in absolutely zero cost for WeldMobilizer's.
template <typename T>
class BodyNodeWelded : public BodyNode<T> {
 public:
  BodyNodeWelded(BodyNodeTopology topology,
               const Body<T>* body, const Mobilizer<T>* mobilizer) :
      BodyNode<T>(topology, body, mobilizer) {}

  /// The Jacobian H_PB_W for this kind of mobilizers has zero columns and
  /// therefore this method results in a no-op.
  void UpdateAcrossBodiesSpatialVelocityJacobian(
      const MultibodyTreeContext<T>& context) const final {}

  void UpdateVelocityKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context) const final {};

  void CalcBodySpatialAcceleration_BaseToTip(
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      const Eigen::Ref<const VectorX<T>>& vdot,
      std::vector<SpatialVector<T>>* A_WB_pool) const final {};
};

}  // namespace multibody
}  // namespace drake
