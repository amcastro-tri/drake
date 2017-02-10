#include "drake/multibody/multibody_tree/body_node_impl.h"

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"
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
