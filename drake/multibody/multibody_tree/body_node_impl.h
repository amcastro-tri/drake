#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

template <typename T, int  num_positions, int num_velocities>
class BodyNodeImpl : public BodyNode<T> {
 public:
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = num_positions, nv = num_velocities};
  typedef SpatialVelocityJacobian<T, nv> HMatrix;

  using BodyNode<T>::get_X_PF;
  using BodyNode<T>::get_X_WP;
  using BodyNode<T>::get_X_MB;
  using BodyNode<T>::get_X_FM;

  BodyNodeImpl(BodyNodeTopology topology,
               const Body<T>* body, const Mobilizer<T>* mobilizer) :
      BodyNode<T>(topology, body, mobilizer) {}

  void UpdateAcrossBodiesSpatialVelocityJacobian(
      const MultibodyTreeContext<T>& context) const final;

  const HMatrix& get_H_FM(const PositionKinematicsCache<T>& pc) const {
    return *reinterpret_cast<const HMatrix*>(
        pc.get_H_FM_pool()[this->get_rigid_velocities_start()].data());
  }

  HMatrix& get_mutable_H_FM(PositionKinematicsCache<T>* pc) const {
    return *reinterpret_cast<HMatrix*>(
        pc->get_mutable_H_FM_pool()[
            this->get_rigid_velocities_start()].mutable_data());
  }

  const HMatrix& get_H_PB_W(const PositionKinematicsCache<T>& pc) const {
    return HMatrix::View(
        pc.get_H_PB_W_pool()[this->get_rigid_velocities_start()].data());
    //return *reinterpret_cast<const HMatrix*>(
    //    pc.get_H_PB_W_pool()[
    //        this->get_num_rigid_velocities()].data());
  }

  HMatrix& get_mutable_H_PB_W(PositionKinematicsCache<T>* pc) const {
    auto& pool = pc->get_mutable_H_PB_W_pool();
    DRAKE_ASSERT(
        this->get_rigid_velocities_start() < static_cast<int>(pool.size()));
    return HMatrix::MutableView(
        pool[this->get_rigid_velocities_start()].mutable_data());
  }
};

}  // namespace multibody
}  // namespace drake
