#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/joint.h"
#include "drake/multibody/multibody_tree/multibody_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"
#include "drake/multibody/multibody_tree/spatial_algebra.h"

namespace drake {
namespace multibody {

template <typename T, int  nq, int nv, bool BequalsM = false>
class JointImpl : public Joint<T> {
 public:
  typedef Eigen::Matrix<T, nq, nv> HtMatrix;

  JointImpl(const Body<T>& parent_body, const Body<T>& child_body,
            const Eigen::Isometry3d& X_PF, const Eigen::Isometry3d& X_BM) :
      Joint<T>(parent_body, child_body, X_PF, X_BM) {}

  int get_num_qs() const final { return nq;}
  int get_num_vs() const final { return nv;}

#if 0
  HtMatrix* get_mutable_H_FM(const PositionKinematicsCache<T>& pc) const {
    return &pc.H_FM_pool[topology_.velocity_index];
  }
#endif

  void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const final
  {
    PositionKinematicsCache<T>& pc = context.get_mutable_position_kinematics();
    const auto q = get_positions_slice(context.get_positions());

#if 0
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
#endif
  }
 private:
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
