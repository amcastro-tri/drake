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
#include "drake/multibody/multibody_tree/spatial_inertia.h"

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
  typedef eigen_aligned_std_vector<ShiftOperator<T>> ShiftOperatorPoolType;
  typedef eigen_aligned_std_vector<Vector3<T>> Vector3PoolType;
  typedef eigen_aligned_std_vector<SpatialInertia<T>> SpatialMatrixPoolType;

  X_PoolType& get_mutable_X_FM_pool() {
    return X_FM_pool_;
  }

  H_FM_PoolType& get_mutable_H_FM_pool() {
    return H_FM_pool_;
  }

  const Isometry3<T>& get_X_FM(BodyNodeIndex body_id) const {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return X_FM_pool_[body_id];
  }

  Isometry3<T>& get_mutable_X_FM(BodyNodeIndex body_id) {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
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

  /// Returns a constant reference to the pose `X_MB` of the node body frame `B`
  /// as measured and expressed in the "mobilized" frame `M`.
  /// In general `X_MB(qf_B)` is a function of the flexible degrees of freedom
  /// of the node body `B`.
  /// @param[in] body_node_id The unique identifier for the computational
  ///                         BodyNode object associated with body `B`.
  /// @returns `X_MB` the pose of the the body frame `B` measured and
  ///                 expressed in the "mobilized" frame `M` .
  const Isometry3<T>& get_X_MB(BodyNodeIndex body_node_id) const {
    DRAKE_ASSERT(0 <= body_node_id && body_node_id < num_nodes_);
    return X_MB_pool_[body_node_id];
  }

  /// Returns a mutable reference to the pose `X_MB` of the node body frame `B`
  /// as measured and expressed in the "mobilized" frame `M`.
  /// In general `X_MB(qf_B)` is a function of the flexible degrees of freedom
  /// of the node body `B`.
  /// @param[in] body_node_id The unique identifier for the computational
  ///                         BodyNode object associated with body `B`.
  /// @returns `X_MB` the pose of the the body frame `B` measured and
  ///                 expressed in the "mobilized" frame `M` .
  Isometry3<T>& get_mutable_X_MB(BodyNodeIndex body_node_id) {
    DRAKE_ASSERT(0 <= body_node_id && body_node_id < num_nodes_);
    return X_MB_pool_[body_node_id];
  }

  /// Returns a constant reference to the pose `X_WB` of the body `B`
  /// (associated with node @p body_node_id) as measured and expressed in the
  /// world frame `W`.
  /// @param[in] body_node_id The unique identifier for the computational
  ///                         BodyNode object associated with body `B`.
  /// @returns `X_WB` the pose of the the body frame `B` measured and
  ///                 expressed in the world frame `W`.
  const Isometry3<T>& get_X_WB(BodyNodeIndex body_node_id) const {
    DRAKE_ASSERT(0 <= body_node_id && body_node_id < num_nodes_);
    return X_WB_pool_[body_node_id];
  }

  /// Returns a mutable reference to the pose `X_WB` of the body `B`
  /// (associated with node @p body_node_id) as measured and expressed in the
  /// world frame `W`.
  /// @param[in] body_node_id The unique identifier for the computational
  ///                         BodyNode object associated with body `B`.
  /// @returns `X_WB` the pose of the the body frame `B` measured and
  ///                 expressed in the world frame `W`.
  Isometry3<T>& get_mutable_X_WB(BodyNodeIndex body_node_id) {
    DRAKE_ASSERT(0 <= body_node_id && body_node_id < num_nodes_);
    return X_WB_pool_[body_node_id];
  }

  /// Returns a mutable reference to the pose `X_PB` of the body frame `B`
  /// as measured and expressed in its parent body frame `P`
  /// @param[in] body_node_id The unique identifier for the computational
  ///                         BodyNode object associated with body `B`.
  /// @returns `X_PB` a mutable reference to the pose of the the body frame `B`
  ///                 measured and expressed in the parent body frame `P`.
  Isometry3<T>& get_mutable_X_PB(BodyNodeIndex body_node_id) {
    DRAKE_ASSERT(0 <= body_node_id && body_node_id < num_nodes_);
    return X_PB_pool_[body_node_id];
  }

  const Isometry3<T>& get_X_PB(BodyNodeIndex body_node_id) const {
    DRAKE_ASSERT(0 <= body_node_id && body_node_id < num_nodes_);
    return X_PB_pool_[body_node_id];
  }

  ShiftOperator<T>& get_mutable_phi_PB_W(BodyIndex body_id) {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return phi_PB_W_pool_[body_id];
  }

  const Vector3<T>& get_com_W(BodyIndex body_id) const {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return com_W_pool_[body_id];
  }

  Vector3<T>& get_mutable_com_W(BodyIndex body_id) {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return com_W_pool_[body_id];
  }

  const SpatialInertia<T>& get_M_Bo_W(BodyIndex body_id) const {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return M_Bo_W_pool_[body_id];
  }

  SpatialInertia<T>& get_mutable_M_Bo_W(BodyIndex body_id) {
    DRAKE_ASSERT(0 <= body_id && body_id < num_nodes_);
    return M_Bo_W_pool_[body_id];
  }

  void Allocate(const MultibodyTreeTopology& tree_topology) {
    num_nodes_ = tree_topology.get_num_body_nodes();
    const int num_rigid_velocities = tree_topology.num_rigid_velocities;
    X_FM_pool_.resize(num_nodes_);
    X_FM_pool_[kWorldBodyId] = Matrix4<T>::Constant(
        Eigen::NumTraits<double>::quiet_NaN());  // It should not be used.

    H_FM_pool_.resize(num_rigid_velocities);

    X_BF_pool_.resize(tree_topology.X_BF_pool_size);

    X_MB_pool_.resize(num_nodes_);
    X_MB_pool_[kWorldBodyId] = Matrix4<T>::Constant(
        Eigen::NumTraits<double>::quiet_NaN());  // It should not be used.

    X_WB_pool_.resize(num_nodes_);
    X_WB_pool_[kWorldBodyId] = Isometry3<T>::Identity();

    X_PB_pool_.resize(num_nodes_);
    X_PB_pool_[kWorldBodyId] = Matrix4<T>::Constant(
        Eigen::NumTraits<double>::quiet_NaN());  // It should not be used.

    phi_PB_W_pool_.resize(num_nodes_);
    phi_PB_W_pool_[kWorldBodyId].SetToNaN();  // It should not be used.

    com_W_pool_.resize(num_nodes_);
    com_W_pool_[kWorldBodyId].setZero();

    M_Bo_W_pool_.resize(num_nodes_);
    M_Bo_W_pool_[kWorldBodyId].SetToNaN();
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
  int num_nodes_{0};
  X_PoolType X_FM_pool_;                 // Indexed by BodyNodeIndex.
  H_FM_PoolType H_FM_pool_;              // Indexed by velocity_start.
  X_PoolType X_BF_pool_;                 // Indexed by X_BF_index.
  X_PoolType X_MB_pool_;                 // Indexed by BodyNodeIndex.
  X_PoolType X_WB_pool_;                 // Indexed by BodyNodeIndex.
  X_PoolType X_PB_pool_;                 // Indexed by BodyNodeIndex.
  ShiftOperatorPoolType phi_PB_W_pool_;  // Indexed by BodyIndex.
  Vector3PoolType com_W_pool_;           // Indexed by BodyIndex.
  SpatialMatrixPoolType M_Bo_W_pool_;    // Indexed by BodyIndex.
};

}  // namespace multibody
}  // namespace drake
