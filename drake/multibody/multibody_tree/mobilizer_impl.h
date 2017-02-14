#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

// Forward declaration.
template<typename T> class Body;  // Only needed for CreateBodyNode().

namespace drake {
namespace multibody {

template <typename T, int  num_positions, int num_velocities>
class MobilizerImpl : public Mobilizer<T> {
 public:
  using Mobilizer<T>::get_id;

  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = num_positions, nv = num_velocities};

  typedef SpatialVelocityJacobian<T, nv> HMatrix;

  MobilizerImpl(const MaterialFrame<T>& inboard_frame,
                const MaterialFrame<T>& outboard_frame) :
      Mobilizer<T>(inboard_frame, outboard_frame) {}

  int get_num_positions() const final { return nq;}
  int get_num_velocities() const final { return nv;}

  /// Sets the what is considered the _zero_ configuration for this mobilizer.
  /// By default this method sets all degrees of freedom related to this
  /// mobilizer to zero.
  /// In general setting all generalized coordinates to zero does not represent
  /// the _zero_ configuration and it might even not represent a mathematicaly
  /// valid configuration. Consider for instance a QuaternionMobilizer, for
  /// which its _zero_ configuration corresponds to the quaternion [1, 0, 0, 0].
  /// For those cases the specific mobilizers must override this method.
  virtual void set_zero_configuration(MultibodyTreeContext<T>* context) const {
    get_mutable_positions(context).setZero();
  }

  void set_zero_velocities(MultibodyTreeContext<T>* context) const {
    get_mutable_velocities(context).setZero();
  }

  void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const final;

 protected:
  Vector<T, nv>& VelocityView(Eigen::Ref<VectorX<T>> v) const {
    DRAKE_ASSERT(v.size() == nv);
    return *reinterpret_cast<Vector<T, nv>*>(v.data());
  }

  const Vector<T, nq>& get_positions(
      const MultibodyTreeContext<T>& context) const
  {
    DRAKE_ASSERT(topology_.num_positions == nq);
    return *reinterpret_cast<const Vector<T, nq>*>(
        context.get_positions().data() + get_positions_start());
  }

  /// Given a mutable MultibodyTreeContext this method regurns a mutable
  /// reference vector to the portion of the generalized coordinates vector for
  /// the entire MultibodyTree that correspods to this mobilizer.
  /// The returned vector has the proper static size for fast computations.
  Vector<T, nq>& get_mutable_positions(MultibodyTreeContext<T>* context) const {
    DRAKE_ASSERT(topology_.num_positions == nq);
    return *reinterpret_cast<Vector<T, nq>*>(
        context->get_mutable_positions().data() + get_positions_start());
  }

  const Vector<T, nv>& get_velocities(
      const MultibodyTreeContext<T>& context) const
  {
    DRAKE_ASSERT(topology_.num_velocities == nv);
    return *reinterpret_cast<const Vector<T, nv>*>(
        context.get_velocities().data() + get_velocities_start());
  }

  Vector<T, nq>& get_mutable_velocities(MultibodyTreeContext<T>* context) const
  {
    return *reinterpret_cast<Vector<T, nv>*>(
        context->get_mutable_velocities().data() + get_velocities_start());
  }

  const Isometry3<T>& get_X_FM(const PositionKinematicsCache<T>& pc) const {
    return pc.get_X_FM(topology_.body_node);
  }

  Isometry3<T>& get_mutable_X_FM(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_FM(topology_.body_node);
  }

  HMatrix& get_mutable_H_FM(PositionKinematicsCache<T>* pc) const {
    return *reinterpret_cast<HMatrix*>(
        pc->get_mutable_H_FM_pool()[topology_.velocities_start].mutable_data());
  }

  std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const BodyNodeTopology& topology,
      const Body<T>* body, const Mobilizer<T>* mobilizer) const final;

 private:
  using Mobilizer<T>::topology_;
  using Mobilizer<T>::get_positions_start;
  using Mobilizer<T>::get_velocities_start;

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

};

}  // namespace multibody
}  // namespace drake
