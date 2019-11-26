#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {  

/// This class is one of the cache entries in MultibodyTreeContext. It holds the
/// results of computations that are used in the recursive implementation of the
/// articulated body algorithm.
///
/// Articulated body algorithm cache entries include:
/// - The articulated body inertia residual force `Zplus_PB_W` for this body
///   projected across its inboard mobilizer to frame P.
/// - The bias spatial acceleration `Ab_WB` for body B including centrifugal
///   and Coriolis terms due to the motion of P in W and of B in P.
///   This bias allows to write:
///     A_WB = Φᵀ(p_PB)A_PB + H_PB vdot_B + Ab_WB(w_WP, V_PB)
/// - The articulated body inertia innovations generalized force `e_B` for this
///   body's mobilizer.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
template<typename T>
class ArticulatedBodyForceBiasCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ArticulatedBodyForceBiasCache)

  /// Constructs an articulated body algorithm entry for the given
  /// MultibodyTreeTopology.
  explicit ArticulatedBodyForceBiasCache(
      const MultibodyTreeTopology& topology) :
      num_nodes_(topology.num_bodies()) {
    Allocate();
  }

  /// The articulated body inertia residual force `Zplus_PB_W` for this body
  /// projected across its inboard mobilizer to frame P.
  const SpatialForce<T>& get_Zplus_PB_W(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Zplus_PB_W_[body_node_index];
  }

  /// Mutable version of get_Zplus_PB_W().
  SpatialForce<T>& get_mutable_Zplus_PB_W(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Zplus_PB_W_[body_node_index];
  }

  /// The Coriolis spatial acceleration `Ab_WB` for this body due to the
  /// relative velocities of body B and body P.
  const SpatialAcceleration<T>& get_Ab_WB(
      BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Ab_WB_[body_node_index];
  }

  /// Mutable version of get_Ab_WB().
  SpatialAcceleration<T>& get_mutable_Ab_WB(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return Ab_WB_[body_node_index];
  }

  /// The articulated body inertia innovations generalized force `e_B` for this
  /// body's mobilizer.
  const VectorUpTo6<T>& get_e_B(BodyNodeIndex body_node_index) const {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return e_B_[body_node_index];
  }

  /// Mutable version of get_e_B().
  VectorUpTo6<T>& get_mutable_e_B(BodyNodeIndex body_node_index) {
    DRAKE_ASSERT(0 <= body_node_index && body_node_index < num_nodes_);
    return e_B_[body_node_index];
  }

 private:
  // The type of the pools for storing spatial forces.
  typedef std::vector<SpatialForce<T>> SpatialForce_PoolType;

  // The type of the pools for storing spatial accelerations.
  typedef std::vector<SpatialAcceleration<T>> SpatialAcceleration_PoolType;

  // The type of the pools for storing vectors up to 6x1.
  typedef std::vector<VectorUpTo6<T>> VectorUpTo6_PoolType;

  // Allocates resources for this articulated body cache.
  void Allocate() {
    Zplus_PB_W_.resize(num_nodes_);
    Ab_WB_.resize(num_nodes_);
    e_B_.resize(num_nodes_);
  }

  // Number of body nodes in the corresponding MultibodyTree.
  int num_nodes_{0};

  // Pools indexed by BodyNodeIndex.
  SpatialForce_PoolType Zplus_PB_W_{};
  SpatialAcceleration_PoolType Ab_WB_{};
  VectorUpTo6_PoolType e_B_{};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ArticulatedBodyForceBiasCache)
