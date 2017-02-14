#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_stl_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

template <typename T>
class VelocityKinematicsCache {
 public:
  typedef std::vector<SpatialVector<T>> HMatrix_PoolType;
  typedef std::vector<SpatialVector<T>> SpatialVelocity_PoolType;

  const VectorX<T>& get_qdot_pool() const { return qdot_;}

  VectorX<T>& get_mutable_qdot_pool() { return qdot_;}
  
  const HMatrix_PoolType& get_Hdot_FM_pool() const {
    return Hdot_FM_pool_;
  }

  HMatrix_PoolType& get_mutable_Hdot_FM_pool() {
    return Hdot_FM_pool_;
  }

  const HMatrix_PoolType& get_Hdot_PB_W_pool() const {
    return Hdot_PB_W_pool_;
  }

  HMatrix_PoolType& get_mutable_Hdot_PB_W_pool() {
    return Hdot_PB_W_pool_;
  }

  /// @returns a constant reference to the spatial velocity of the body node's
  /// body `B` measured in its parent body `P` and expressed in the world
  /// frame `W`.
  const SpatialVector<T>& get_V_PB_W(BodyNodeIndex body_id) const {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return V_PB_W_pool_[body_id];
  }

  /// @returns a mutable reference to the spatial velocity of the body node's
  /// body `B` measured in its parent body `P` and expressed in the world
  /// frame `W`.
  SpatialVector<T>& get_mutable_V_PB_W(BodyNodeIndex body_id) {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return V_PB_W_pool_[body_id];
  }

  const SpatialVector<T>& get_V_WB(BodyNodeIndex body_id) const {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return V_WB_pool_[body_id];
  }

  SpatialVector<T>& get_mutable_V_WB(BodyNodeIndex body_id) {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return V_WB_pool_[body_id];
  }

  void Allocate(const MultibodyTreeTopology& tree_topology) {
    num_nodes_ = tree_topology.get_num_body_nodes();
    num_positions_ = tree_topology.num_rigid_positions;
    const int num_rigid_velocities = tree_topology.num_rigid_velocities;

    qdot_.resize(num_positions_);

    Hdot_FM_pool_.resize(num_rigid_velocities);
    Hdot_PB_W_pool_.resize(num_rigid_velocities);

    V_PB_W_pool_.resize(num_nodes_);
    V_PB_W_pool_[kWorldBodyId].SetToNaN();  // It should not be used.

    V_WB_pool_.resize(num_nodes_);
    V_PB_W_pool_[kWorldBodyId].SetZero();  // World's velocity is always zero.
  }

  void Print() {
    PRINT_VAR(V_PB_W_pool_.size());
    for (const SpatialVector<T>& V_PB_W: V_PB_W_pool_) {
      PRINT_VAR(V_PB_W);
    }
  }

 private:
  int num_nodes_{0};
  int num_positions_{0};
  VectorX<T> qdot_;                      // Indexed by BodyNodeIndex.
  SpatialVelocity_PoolType V_PB_W_pool_; // Indexed by BodyNodeIndex.
  SpatialVelocity_PoolType V_WB_pool_;   // Indexed by BodyNodeIndex.
  HMatrix_PoolType Hdot_FM_pool_;        // Indexed by velocity_start.
  HMatrix_PoolType Hdot_PB_W_pool_;      // Indexed by velocity_start.
};

}  // namespace multibody
}  // namespace drake
