#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_stl_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

#include <iostream>

namespace drake {
namespace multibody {

template <typename T>
class CompositeBodyInertiasCache {
 public:
  typedef eigen_aligned_std_vector<SpatialInertia<T>> SpatialMatrixPoolType;

  const SpatialMatrixPoolType& get_R_Bo_W_pool() const { return R_Bo_W_pool_;}

  SpatialMatrixPoolType& get_mutable_R_Bo_W_pool() { return R_Bo_W_pool_;}

  const SpatialInertia<T>& get_R_Bo_W(BodyIndex body_id) const {
    DRAKE_ASSERT(0 <= body_id && body_id < num_bodies_);
    return R_Bo_W_pool_[body_id];
  }

  SpatialInertia<T>& get_mutable_R_Bo_W(BodyIndex body_id) {
    DRAKE_ASSERT(0 <= body_id && body_id < num_bodies_);
    return R_Bo_W_pool_[body_id];
  }

  void Allocate(const MultibodyTreeTopology& tree_topology) {
    num_bodies_ = tree_topology.get_num_bodies();
    R_Bo_W_pool_.resize(num_bodies_);
  }

  void Print() {
    std::cout << "CompositeBodyInertiasCache::Print()" << std::endl;
    for (BodyIndex body_id(0); body_id < num_bodies_; ++body_id) {
      const SpatialInertia<T>& R_Bo_W = get_R_Bo_W(body_id);
      std::cout << "CBI for body " << body_id << ":" << std::endl;
      std::cout << R_Bo_W << std::endl;
    }
  }

 private:
  int num_bodies_{0};
  // Pool of composite body inertias compute about each body frame origin Bo and
  // expressed in the world frame.
  SpatialMatrixPoolType R_Bo_W_pool_;  // Indexed by BodyIndex.
};

}  // namespace multibody
}  // namespace drake
