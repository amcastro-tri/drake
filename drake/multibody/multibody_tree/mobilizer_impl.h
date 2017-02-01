#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

namespace drake {
namespace multibody {

template <typename T, int  nq, int nv>
class MobilizerImpl : public Mobilizer<T> {
 public:
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {num_positions = nq, num_velocities = nv};

  typedef SpatialVelocityJacobian<T, nv> HMatrix;

  MobilizerImpl(const BodyFrame<T>& inboard_frame,
                const BodyFrame<T>& outboard_frame) :
      Mobilizer<T>(inboard_frame, outboard_frame) {}

  int get_num_positions() const final { return nq;}
  int get_num_velocities() const final { return nv;}

#if 0
  void UpdatePositionKinematics(
      const MultibodyTreeContext<T>& context) const final {

    // Topological information.
    const BodyNodeTopology& node_topology =
        contex.get_body_node_topology(this->get_id());
    const BodyNodeIndex node_id = node_topology.id;

    // Extract const variables from the context.
    auto q = context.get_positions().template segment<nq>(
        node_topology.rigid_positions_start);

    // Extract mutable variables from the cache.
    // Cache entries are indexed by BodyNode id.
    PositionKinematicsCache<T>* pc = context.get_mutable_position_kinematics();
    HMatrix& H_FM = pc->get_mutable_H_FM_pool<nv>(node_id);

    // Perform computations.
    DoCalcAcrossMobilizerVelocityJacobian(q, &H_FM);
  }
#endif

  void UpdatePositionKinematics(
      const MultibodyTreeContext<T>& context) const final;

#if 0
  HtMatrix* get_mutable_H_FM(const PositionKinematicsCache<T>& pc) const {
    return &pc.H_FM_pool[topology_.velocity_index];
  }

  void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const final
  {
    PositionKinematicsCache<T>& pc = context.get_mutable_position_kinematics();
    const auto q = get_positions_slice(context.get_positions());

    get_mutable_X_FM(pc) = CalcAcrossJointTransform(q);
    // Since we are within a base-to-tip recursion loop, X_WP was already
    // computed and, with X_FM, we can now compute X_PB and X_WB.
    CalcBodyPoses(pc, get_mutable_X_PB(pc), get_mutable_X_WB(pc));

    // Computes the across joint Jacobian Ht_FM(q) expressed in F.
    CalcAcrossJointVelocityJacobian(q, get_mutable_Ht_FM(pc));

    // Computes Ht_PB_W, the across joint velocity Jacobian between the parent's
    // body frame P and the child's body frame B expressed in the world's
    // frame W.
    CalcParentToChildJacobianInWorld(pc, get_mutable_Ht_PB_W(pc));
  }
#endif
 private:

#if 0
  // Make NVI?? sot that the pointers are checked already when passed.
  // Not in this case since this method is not virtual but all joints need it.
  // Same for all mobilizers.
  void CalcBodyPoses(const PositionKinematicsCache<T>& pc,
                     Isometry3<T>* mutable_X_PB, Isometry3<T>* mutable_X_WB) const {
    DRAKE_ASSERT(mutable_X_PB != nullptr);
    DRAKE_ASSERT(mutable_X_WB != nullptr);
    const Isometry3d<T>& X_PF = getX_PF();
    const Isometry3d<T>& X_MB = getX_MB();
    // Just computed by UpdatePositionKinematicsCache().
    const Isometry3d<T>& X_FM = getX_FM(pc);
    // Computed as part of the base-to-tip recursion.
    const Isometry3d<T>& X_WP = getX_WP(pc);
    Isometry3d<T>& X_PB = *mutable_X_PB;
    Isometry3d<T>& X_WB = *mutable_X_WB;

    const Isometry3d<T> X_FB = (BequalsM ? X_FM : X_FM * X_MB);
    X_PB = X_PF * X_FB;
    X_WB = X_WP * X_PB;
  }
#endif

#if 0
  void CalcParentToChildJacobianInWorld(
      const PositionKinematicsCache<T>& pc, Ht_PB_W) {
    const HType& Ht_FM = getH_FM(pc);
    const auto& R_PF = getX_PF().linear();  // Orientation of F in P.

    // Available already within a base-to-tip recursion.
    const auto& R_WP = getX_WP(pc).linear();  // Parent orientation in world.
    const auto R_WF = R_WP * R_PF;
    if (BequalsM) {
      // Re-express each column of Ht (a spatial velocity) in the world frame.
      H_PB_W = ReExpressSpatialVelocity(Ht_FM, R_WF);
    } else {
      // Vector from Mo to Bo.
      const auto& r_MB   = getX_MB().translation();  // Expressed in M.
      // Just calculated by CalcAcrossJointVelocityJacobian().
      const auto& R_FM   = getX_FM(pc).linear();
      const Vector3<T> r_MB_F = R_FM * r_MB;  // Expressed in F.

#if 0
      HType Ht_MB_F;
      // How it'd look with Eigen.
      Ht_MB_F.template topRows<3>() = Vector3<T>::Zero();
      Ht_MB_F.template bottomRows<3>().noalias() =
          Ht_FM.template topRows<3>().colwise().cross(r_MB_F);
      r_MB_F.cross(r_MB_F);
      // Ht_PB_W = R_WF * Ht_FB
      Ht_PB_W = R_WF * (Ht_FM + Ht_MB_F);  // Re-express from F to W.
#endif

      // Obtain spatial velocity Ht_FB of frame B with respect to F, expressed
      // in F. Then, express it in world:
      //   Ht_PB_W = R_WF * Ht_FB.
      Ht_PB_W = R_WF * ShiftSpatialVelocity(Ht_FM, r_MB_F);
    }
  }
#endif

  // Some ideas here with API's taking Eigen vectors.
#if 0
  // Defaults to identity. Assumes nq == nv.
  virtual void CalcN(const Ref<const VectorX<T>>& q, Ref<MatrixX<T>> N) override {
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    assert(nq == nv);
    N.setIdentity(nv, nv);
  }
  virtual void MultiplyByN(const Ref<const VectorX<T>>& q, const Ref<const VectorX<T>>& v, Ref<VectorX<T>> N_times_v) override {
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    assert(nq == nv);
    N_times_v.template head<nq>() = v.template head<nq>();
  }
  Isometry3<T> CalcX_FM(const Ref<const VectorX<T>>& q) {
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    return Isometry3<T>::Identity();
  }
#endif

};

}  // namespace multibody
}  // namespace drake
