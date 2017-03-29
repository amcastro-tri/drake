#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra_old.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"
#include "drake/multibody/multibody_tree/velocity_kinematics_cache.h"

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

  BodyNodeImpl(const MultibodyTree<T>* tree, const BodyNodeTopology& topology,
               const Body<T>* body, const Mobilizer<T>* mobilizer) :
      BodyNode<T>(topology, body, mobilizer) {
    this->set_parent_tree(tree);
  }

  void Compile() final {};

  void UpdateAcrossBodiesSpatialVelocityJacobian(
      const MultibodyTreeContext<T>& context) const final;

  /// This method can anly be called within a base-to-tip loop.
  void UpdateVelocityKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context) const;

  void CalcBodySpatialAcceleration_BaseToTip(
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      const Eigen::Ref<const VectorX<T>>& vdot,
      std::vector<GeneralSpatialVector<T>>* A_WB_pool) const final;

  /// Helper methods access the context.
  const Vector<T, nv>& get_mobilizer_velocities(
      const MultibodyTreeContext<T>& context) const
  {
    //return *reinterpret_cast<const Vector<T, nv>*>(
    //    context.get_velocities().data() + this->get_rigid_velocities_start());
    return get_mobilizer_velocities_from_pool(context.get_velocities());
  }

  /// Helper methods to access node quantities as expressions of fixed size.
  const Vector<T, nv>& get_mobilizer_velocities_from_pool(
      const VectorX<T>& vdot_pool) const {
    return *reinterpret_cast<const Vector<T, nv>*>(
        &(vdot_pool[this->get_rigid_velocities_start()]));
  };

  /// Helper Methods to access the position kinematics cache.

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

  // Helper methods to extract entries from the velocity kinematics cache.

  /// @returns a mutable reference to the vector of time derivatives of the
  /// mobilizer generalized positions `qm` corresponding to this node's
  /// mobilizer.
  Vector<T, nq>& get_mutable_qmdot(VelocityKinematicsCache<T>* vc) const {
    DRAKE_ASSERT(this->get_num_rigid_positions() == nq);
    return *reinterpret_cast<Vector<T, nq>*>(
        vc->get_mutable_qdot_pool().data() + this->get_rigid_positions_start());
  }

  const Vector<T, nv>& get_qmdot(const VelocityKinematicsCache<T>& vc) const
  {
    DRAKE_ASSERT(this->get_num_rigid_velocities() == nv);
    return *reinterpret_cast<const Vector<T, nv>*>(
        vc.get_qdot_pool().data() + this->get_rigid_positions_start());
  }

  const GeneralSpatialVector<T>& get_V_PB_W(const VelocityKinematicsCache<T> vc) const {
    return vc.get_V_PB_W(this->get_index());
  }

  GeneralSpatialVector<T>& get_mutable_V_PB_W(VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_PB_W(this->get_index());
  }

  const GeneralSpatialVector<T>& get_V_WB(const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_WB(this->get_index());
  }

  GeneralSpatialVector<T>& get_mutable_V_WB(VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_WB(this->get_index());
  }

  /// @returns the spatial velocity `V_WP` of the body `P` in the parent node.
  const GeneralSpatialVector<T>& get_V_WP(const VelocityKinematicsCache<T>& vc) const {
    return vc.get_V_WB(this->topology_.parent_body_node);
  }

 protected:
  using BodyNode<T>::body_;
  using BodyNode<T>::mobilizer_;
};

}  // namespace multibody
}  // namespace drake
