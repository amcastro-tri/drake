#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_stl_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"
#include "drake/multibody/multibody_tree/mobilizer_context.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;


namespace drake {
namespace multibody {

template <typename T>
class PositionKinematicsCache {
 public:
  typedef eigen_aligned_std_vector<SpatialVector<T>> H_FM_PoolType;
  typedef eigen_aligned_std_vector<Isometry3<T>> X_PoolType;

  X_PoolType& get_mutable_X_FM_pool() {
    return X_FM_pool_;
  }

  H_FM_PoolType& get_mutable_H_FM_pool() {
    return H_FM_pool_;
  }

  const Isometry3<T>& get_X_FM(BodyNodeIndex body_id) const {
    return X_FM_pool_[body_id];
  }

  Isometry3<T>& get_mutable_X_FM(BodyNodeIndex body_id) {
    return X_FM_pool_[body_id];
  }

  int get_X_BF_pool_size() const { return static_cast<int>(X_BF_pool_.size());}
  const X_PoolType& get_X_BF_pool() const { return X_BF_pool_;}
  X_PoolType& get_mutable_X_BF_pool() { return X_BF_pool_;}
  const Isometry3<T>& get_X_BF(int index) const { return X_BF_pool_[index];}
  Isometry3<T>& get_mutable_X_BF(int index) {
    DRAKE_ASSERT( 0 <= index && index < get_X_BF_pool_size());
    return X_BF_pool_[index];
  }

  void Allocate(const MultibodyTreeTopology& tree_topology) {
    const int num_nodes = tree_topology.get_num_body_nodes();
    const int num_rigid_velocities = tree_topology.num_rigid_velocities;
    X_FM_pool_.resize(num_nodes);
    X_FM_pool_[kWorldBodyId].setIdentity();

    H_FM_pool_.resize(num_rigid_velocities);

    X_BF_pool_.resize(tree_topology.X_BF_pool_size);
  }

  void Print() {
    PRINT_VAR(X_FM_pool_.size());
    for (const Isometry3<T>& X_FM: X_FM_pool_) {
      PRINT_VARn(X_FM.matrix());
    }

    PRINT_VAR(H_FM_pool_.size());
    for (const auto& H_FM: H_FM_pool_) {
      PRINT_VAR(H_FM);
    }
  }

 private:
  X_PoolType X_FM_pool_;  // Indexed by BodyNodeIndex.
  H_FM_PoolType H_FM_pool_;  // Indexed by velocity_start.
  X_PoolType X_BF_pool_;  // Indexed by X_BF_index.
};

}  // namespace multibody
}  // namespace drake
