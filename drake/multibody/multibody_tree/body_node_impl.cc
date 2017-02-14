#include "drake/multibody/multibody_tree/body_node_impl.h"

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"
#include "drake/multibody/multibody_tree/velocity_kinematics_cache.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

namespace drake {
namespace multibody {

template <typename T, int  nq, int nv>
void BodyNodeImpl<T, nq, nv>::UpdateAcrossBodiesSpatialVelocityJacobian(
    const MultibodyTreeContext<T>& context) const {
  PositionKinematicsCache<T>* pc = context.get_mutable_position_kinematics();

  // Jacobian for the spatial velocity V_FM representing the spatial velocity
  // of frame M measured and expressed in F.
  // F: Mobilizer inboard frame.
  // M: Mobilizer outboard frame.
  // Previously computed by Mobilizer::UpdatePositionKinematicsCache().
  const HMatrix& H_FM = get_H_FM(*pc);

  // P: Parent body frame.
  // F: Mobilizer inboard frame.
  const Isometry3<T>& X_PF = get_X_PF(*pc);

  // Pose of the parent body P in world frame W.
  // Available since we are called within a base-to-tip recursion.
  const Isometry3<T>& X_WP = get_X_WP(pc);

  // Orientation (rotation) of frame F with respect to the world frame W.
  const Matrix3<T> R_WF = X_WP.rotation() * X_PF.rotation();

  // Output:
  // H_PB_W
  HMatrix& H_PB_W = get_mutable_H_PB_W(pc);

  // In terms of spatial velocities (each column of H is a spatial velocity),
  // For a RIGID parent body P:
  //    - V_PB = V_FB,  since V_PF = 0.
  // For a RIGID node body B, we can use rigid shift operator:
  //    - V_FB = phiT_MB * V_FM
  // And therefore:
  //    - V_PB = V_FB = phiT_MB * V_FM
  // Or in terms of velocity Jacobians:
  //    - H_PB = phiT_MB * H_FM
  //
  // It is convenient now to perform this last computations in the F frame
  // given the available pre-computed quantities.
  // Vector from Mo to Bo expressed in frame F:
  const Vector3<T> p_MB_F =
      get_X_FM(pc).rotation() * get_X_MB(pc).translation();
  const ShiftOperator<T> phiT_MB_F(p_MB_F);

  // Perform H_PB = phiT_MB * H_FM in the F frame and re-express in the
  // world frame W.
  H_PB_W = R_WF * (phiT_MB_F.transpose() * H_FM);
}

template <typename T, int  nq, int nv>
void BodyNodeImpl<T, nq, nv>::UpdateVelocityKinematicsCache_BaseToTip(
    const MultibodyTreeContext<T>& context) const {
  // This method should not be called for the "world" body node.
  DRAKE_ASSERT(this->topology_.body != kWorldBodyId);

  const PositionKinematicsCache<T>& pc = context.get_position_kinematics();
  VelocityKinematicsCache<T>* vc = context.get_mutable_velocity_kinematics();

  // Given the generalize velocity v, update qdot for each mobilizer.
  // qdot = N(q) * v
  mobilizer_->CalcQDot(context, get_mutable_qmdot(vc));

  // Generalized velocities local to this node's mobilizer.
  const Vector<T, nv>& vm = this->get_mobilizer_velocities(context);

  get_mutable_V_PB_W(vc) = get_H_PB_W(pc) * vm;

  // TODO(amcastro-tri): compute Hdot_FM
  // TODO(amcastro-tri): compute Hdot_PB_W

  // Update velocity V_WB of this body's node in the world frame.
  const SpatialVector<T>& V_WP = get_V_WP(*vc);
  const SpatialVector<T>& V_PB_W = get_V_PB_W(*vc);
  const ShiftOperatorTranspose<T>& ST_PB_W = this->get_phi_PB_W(pc).transpose();
  get_mutable_V_WB(vc) = ST_PB_W * V_WP + V_PB_W;
}

// Macro used to explicitly instantiate implementations on all sizes needed.
#define EXPLICITLY_INSTANTIATE(T) \
template class BodyNodeImpl<T, 1, 1>; \
template class BodyNodeImpl<T, 2, 2>; \
template class BodyNodeImpl<T, 3, 3>; \
template class BodyNodeImpl<T, 4, 4>; \
template class BodyNodeImpl<T, 5, 5>; \
template class BodyNodeImpl<T, 6, 6>; \
template class BodyNodeImpl<T, 7, 6>;

// Explicitly instantiates on the most common scalar types.
EXPLICITLY_INSTANTIATE(double);

}  // namespace multibody
}  // namespace drake
