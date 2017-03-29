#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra_old.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <typename T>
class BodyNode : public MultibodyTreeElement<BodyNode<T>, BodyNodeIndex> {
 public:
  BodyNode(BodyNodeTopology topology,
           const Body<T>* body, const Mobilizer<T>* mobilizer) :
      topology_(topology), body_(body), mobilizer_(mobilizer) {
    DRAKE_ASSERT(body != nullptr);
    DRAKE_ASSERT(!(mobilizer == nullptr && body->get_index() != kWorldBodyId));
  }

  BodyNodeIndex get_index() const final { return topology_.id;}

  int get_rigid_positions_start() const
  {
    return topology_.rigid_positions_start;
  }

  int get_num_rigid_positions() const
  {
    return topology_.num_rigid_positions;
  }

  int get_rigid_velocities_start() const
  {
    return topology_.rigid_velocities_start;
  }

  int get_num_rigid_velocities() const
  {
    return topology_.num_rigid_velocities;
  }

  const Body<T>& get_body() const {
    DRAKE_ASSERT(get_body_id().is_valid());
    return this->get_parent_tree().get_body(get_body_id());
  }

  const Body<T>& get_parent_body() const {
    DRAKE_ASSERT(get_parent_body_id().is_valid());
    return this->get_parent_tree().get_body(get_parent_body_id());
  }

  const Mobilizer<T>& get_mobilizer() const {
    DRAKE_ASSERT(get_mobilizer_id().is_valid());
    return this->get_parent_tree().get_mobilizer(get_mobilizer_id());
  }

  MobilizerIndex get_mobilizer_id() const { return topology_.mobilizer;}

  BodyIndex get_body_id() const { return topology_.body;}

  BodyIndex get_parent_body_id() const { return topology_.parent_body;}

  /// This method can anly be called within a base-to-tip loop.
  void UpdatePositionKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context) const {
    // This method should not be called for the "world" body node.
    DRAKE_ASSERT(topology_.body != kWorldBodyId);

    // This computes into the PositionKinematicsCache:
    // - X_PB(qf_P, qr_B, qf_B)
    // - X_WB(q(W:P), qf_P, qr_B, qf_B)
    // It assumes:
    // - The body for this node updates its attached frame poses X_BF(qf_B).
    // - We are in a base-to-tip recursion and therefore X_PF(qf_P) and X_WP are
    //   available.
    CalcAcrossMobilizerBodyPoses(context);

    // Update Body specific kinematics. These are:
    // - phi_PB_W: shift operator from P to B.
    // - com_W: center of mass.
    // - M_Bo_W: Spatial inertia.
    UpdateBodySpecificKinematicsCache_BaseToTip(context);

    // With H_FM(qr) already in the cache (computed by
    // Mobilizer::UpdatePositionKinematicsCache()) this call updates the cache
    // entries for H_PB_W, the Jacobian for the SpatialVelocity jump between
    // body B and its parent body P expressed in the world frame W.
    UpdateAcrossBodiesSpatialVelocityJacobian(context);
  }

  /// This method can anly be called within a base-to-tip loop.
  virtual void UpdateVelocityKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context) const = 0;

  /// Computes the composite body inertia (CBI) for this node's body.
  /// By definition, the CBI of a body is the composition of its
  /// children's CBI's.
  void CalcCompositeBodyInertia_TipToBase(
      const PositionKinematicsCache<T>& pc,
      eigen_aligned_std_vector<SpatialInertia<T>>& cbi_array)
  {
    SpatialInertia<T>& R_Bo_W = cbi_array[topology_.body];
    R_Bo_W = get_M_Bo_W(pc);  // Initialize to this node's body inertia.
    for (BodyIndex child: topology_.child_bodies) {
      // Spatial inertia computed about the child's frame origin Co,
      // expressed in world.
      // Already computed in the next level within the tip-to-base recursion.
      const SpatialInertia<T>& R_Co_W = cbi_array[child];
      // Vector from the child body Co (the B) to this body Bo (the P).
      const Vector3<T>& p_CoBo_W = -pc.get_phi_PB_W(child).offset();
      // Shifts from Co to Bo and adds it to the total CBI for B.
      R_Bo_W += R_Co_W.Shift(p_CoBo_W);
    }
  }

  virtual void CalcBodySpatialAcceleration_BaseToTip(
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      const Eigen::Ref<const VectorX<T>>& vdot,
      std::vector<GeneralSpatialVector<T>>* A_WB_pool) const = 0;

  void PrintTopology() const {
    std::cout << "BodyNode id: " << topology_.id << std::endl;
    std::cout << "  Level: " << topology_.level << std::endl;
    std::cout << "  Mobilizer: " << topology_.mobilizer << std::endl;
    std::cout << "  Body: " << topology_.body << std::endl;
    if (topology_.get_num_children() != 0) {
      std::cout << "Children ( " <<
                topology_.get_num_children() << "): ";
      std::cout << topology_.child_bodies[0];
      for (int ichild = 1; ichild < topology_.get_num_children(); ++ichild) {
        std::cout << ", " << topology_.child_bodies[ichild];
      }
      std::cout << std::endl;
    }
    std::cout << "  Parent BodyNode: " <<
              topology_.parent_body_node << std::endl;
    std::cout << "  Num(qr): " << topology_.num_rigid_positions << std::endl;
    std::cout << "  Num(vr): " << topology_.num_rigid_velocities << std::endl;
    std::cout << "  Num(qf): " <<
              topology_.num_flexible_positions << std::endl;
    std::cout << "  Num(vf): " <<
              topology_.num_flexible_velocities << std::endl;
    std::cout << "  qr_start: " <<
              topology_.rigid_positions_start << std::endl;
    std::cout << "  vr_start: " <<
              topology_.rigid_velocities_start << std::endl;
    std::cout << "  qf_start: " <<
              topology_.flexible_positions_start << std::endl;
    std::cout << "  vf_start: " <<
              topology_.flexible_velocities_start << std::endl;
    std::cout << "  X_PF_index: " << topology_.X_PF_index << std::endl;
    std::cout << "  F_equals_P: " <<
              (topology_.F_equals_P ? "true" : "false") << std::endl;
  }

 protected:
  BodyNodeTopology topology_;
  // Pointers for fast access.
  const Body<T>* body_{nullptr};
  const Mobilizer<T>* mobilizer_{nullptr};

  // Helper methods to extract entries from the context.

  // Get from the position kinematics cache a constant reference to the pose
  // X_PF of the "fixed" frame F as measured and expressed in the inboard
  // (parent) body frame P.
  const Isometry3<T>& get_X_PF(const PositionKinematicsCache<T>& pc) const {
    const auto& pool = pc.get_X_BF_pool();
    DRAKE_ASSERT(topology_.X_PF_index < static_cast<int>(pool.size()));
    return pool[topology_.X_PF_index];
  }

  Isometry3<T>& get_mutable_X_PF(PositionKinematicsCache<T>* pc) const {
    auto& pool = pc->get_mutable_X_BF_pool();
    DRAKE_ASSERT(topology_.X_PF_index < static_cast<int>(pool.size()));
    return pool[topology_.X_PF_index];
  }

  // Get from the position kinematics cache a constant reference to the pose
  // `X_MB` of the node body frame `B` as measured and expressed in the
  // "mobilized" frame `M`.
  /// In general `X_MB(qf_B)` is a function of the flexible degrees of freedom
  // of the node body `B`.
  // @param[in] body_node_id The unique identifier for the computational
  //                         BodyNode object associated with body `B`.
  // @returns `X_MB` the pose of the the body frame `B` measured and
  //                 expressed in the "mobilized" frame `M` .
  const Isometry3<T>& get_X_MB(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_MB(topology_.id);
  }

  const Isometry3<T>& get_X_FM(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_FM(topology_.id);
  }

  const Isometry3<T>& get_X_WP(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_WB(topology_.parent_body_node);
  }

  const Isometry3<T>& get_X_PB(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_PB(topology_.id);
  }

  Isometry3<T>& get_mutable_X_PB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_PB(topology_.id);
  }

  const Isometry3<T>& get_X_WB(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_WB(topology_.id);
  }

  Isometry3<T>& get_mutable_X_WB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_WB(topology_.id);
  }

  /// Returns a constant reference to the rigid body shift operator between
  /// this node's body `B` and its parent `P`.
  /// @param[in] pc The position kinematics cache.
  /// @returns phi_PB_W The rigid body shift operator between this node's body
  ///                   `B` and its parent `P` expressed in the world frame `W`.
  const ShiftOperator<T>& get_phi_PB_W(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_phi_PB_W(topology_.body);
  }

  ShiftOperator<T>& get_mutable_phi_PB_W(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_phi_PB_W(topology_.body);
  }

  Vector3<T>& get_mutable_com_W(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_com_W(topology_.body);
  }

  const SpatialInertia<T>& get_M_Bo_W(
      const PositionKinematicsCache<T>& pc) const {
    return pc.get_M_Bo_W(topology_.body);
  }

  SpatialInertia<T>& get_mutable_M_Bo_W(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_M_Bo_W(topology_.body);
  }

  void CalcAcrossMobilizerBodyPoses(
      const MultibodyTreeContext<T>& context) const {
    PositionKinematicsCache<T>* pc = context.get_mutable_position_kinematics();

    // Body for this node.
    const Body<T>& BodyB = get_body();

    // Body for this node's parent, or the parent body P.
    const Body<T>& BodyP = get_parent_body();

    // Inboard/Outboard frames of this node's mobilizer.
    const MaterialFrame<T>& FrameF = get_mobilizer().get_inboard_frame();
    DRAKE_ASSERT(FrameF.get_body_id() == BodyP.get_index());
    const MaterialFrame<T>& FrameM = get_mobilizer().get_outboard_frame();
    DRAKE_ASSERT(FrameM.get_body_id() == BodyB.get_index());

    // Input (const):
    // - X_PF(qf_P)
    // - X_MB(qf_B)
    // - X_FM(qr_B)
    // - X_WP(q(W:B), where q(W:B) includes all positions in the kinematics path
    //                from body B to the world W.
    const Isometry3<T>& X_MB = get_X_MB(pc);
    const Isometry3<T>& X_FM = get_X_FM(pc);
    const Isometry3<T>& X_WP = get_X_WP(pc);

    // Output (updating a cache entry):
    // - X_PB(qf_P, qr_B, qf_B)
    // - X_WB(q(W:P), qf_P, qr_B, qf_B)
    Isometry3<T>& X_PB = get_mutable_X_PB(pc);
    Isometry3<T>& X_WB = get_mutable_X_WB(pc);

    // TODO(amcastro-tri): Consider logic for the common case B = M.
    // In that case X_FB = X_FM as suggested by setting X_MB = Id.
    const Isometry3<T> X_FB = X_FM * X_MB;

    // Given the pose X_FB of body frame B measured in the mobilizer inboard
    // frame F, we can ask frame F (who's parent body is P) for the pose of body
    // B measured in the frame of the parent body P.
    // In the particular case F = B, this method directly returns X_FB.
    // For flexible bodies this gives the chance to frame F to pull its pose
    // from the context.
    X_PB = FrameF.get_offset_pose_in_body(context, X_FB);

    X_WB = X_WP * X_PB;
  }

  void UpdateBodySpecificKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context) const {
    PositionKinematicsCache<T> *pc = context.get_mutable_position_kinematics();

    // Just computed as part of a base-to-tip recursion by
    // CalcAcrossMobilizerBodyPoses().
    const Isometry3<T>& X_WB = get_X_WB(pc);

    // Output into the PositionKinematicsCache:
    ShiftOperator<T>& phi_PB_W = get_mutable_phi_PB_W(pc);
    Vector3<T>& com_W = get_mutable_com_W(pc);
    SpatialInertia<T>& M_Bo_W = get_mutable_M_Bo_W(pc);

    // Shift operator between the parent body P and this node's body B,
    // expressed in the world frame W.
    phi_PB_W = ShiftOperator<T>(
        /* p_PB_W = R_WP * p_PB */
        get_X_WP(pc).rotation() * get_X_PB(pc).translation());

    // Compute center of mass Bc of body B measured and expressed in B.
    Vector3<T> com_B = body_->CalcCenterOfMassInBodyFrame(context);

    // com_W = p_WP + R_WB * p_BBc_B = p_WP + p_BBc_W.
    com_W = X_WB * com_B;

    // Compute the rotational (unit) inertia for this node's body in the
    // world frame.
    UnitInertia<T> G_Bo_B = body_->CalcUnitInertiaInBodyFrame(context);
    UnitInertia<T> G_Bo_W = G_Bo_B.ReExpress(X_WB.rotation());

    // Spatial inertia of this node's body in the world frame.
    M_Bo_W = SpatialInertia<T>(body_->CalcMass(context), com_W, G_Bo_W);
  }

  virtual void UpdateAcrossBodiesSpatialVelocityJacobian(
      const MultibodyTreeContext<T>& context) const = 0;
};

}  // namespace multibody
}  // namespace drake
