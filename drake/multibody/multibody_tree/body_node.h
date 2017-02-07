#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

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
    DRAKE_ASSERT(!(mobilizer == nullptr && body->get_id() != kWorldBodyId));
  }

#if 0
  BodyNode(BodyIndex body_id, MobilizerIndex mobilizer_id) :
      body_id_(body_id), mobilizer_id_(mobilizer_id) {}

  void SetArrayIndexes(int position_start, int velocity_start,
                       int num_rigid_positions, int num_flexible_positions,
                       int num_rigid_velocities, int num_flexible_velocities) {
    // Positions are arranged first: Rigid DOF's followed by flexible DOF's.
    rigid_positions_start_ = position_start;
    num_rigid_positions_ = num_rigid_positions;
    flexible_positions_start_ = position_start + num_rigid_positions;
    num_flexible_positions_ = num_flexible_positions;

    // Velocities follow positions: Rigid DOF's followed by flexible DOF's.
    rigid_velocities_start_ = velocity_start;
    num_rigid_velocities_ = num_rigid_velocities;     
    flexible_velocities_start_ = velocity_start + num_rigid_velocities;
    num_flexible_velocities_ = num_flexible_velocities;
  }
#endif

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

  const Body<T>& get_body() const {
    DRAKE_ASSERT(body_ != nullptr);
    return *body_;
  }

  const Mobilizer<T>& get_mobilizer() const {
    DRAKE_ASSERT(mobilizer_ != nullptr);
    return *mobilizer_;
  }

  MobilizerIndex get_mobilizer_id() const { return topology_.mobilizer;}

  BodyIndex get_body_id() const { return topology_.body;}

  /// Computes the rigid body inertia matrix for a given, fixed, value of the
  /// flexible generalized coordinates @p qf.
  //virtual SpatialMatrix DoCalcSpatialInertia(const VectorX<T>& qf) const = 0;

  // void CalcPositionsKinematics(context, PositionKinematics<T> some_output_structure).
  // UpdatePostionKinematics calls CalcPositionKinematics.

  void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const {
    //const Mobilizer<T>& mobilizer = get_mobilizer();
    //const Body<T>& body = get_body();

    CalcAcrossMobilizerBodyPoses(context);

#if 0
    // Update position kinematics that depend on mobilizers only:
    // - X_FM(q), H_FM(q), HdotTimesV_FM(q)
    mobilizer.UpdatePositionKinematicsCache(context);

    // Update all body frames attached this body.
    // In particular, the pose X_BM of the inboard frame M in B will be
    // immediately used below to compute across mobilizer transforms between
    // body frames concluding with the update of the body pose X_WB.
    // The outboard frames attached to this body will be used in the
    // computations at the next tree level.
    // These body frame updates depend in general of the flexible generalized
    // coordinates of the body.
    body.UpdateAttachedBodyFrames(context);

    // Perform across-body computations. These couple rigid motions given by
    // the mobilizer connecting inboard and outboard bodies and the flexible
    // motions of those two bodies.
    // These computations assume a base-to-tip recursion. They are generic,
    // independent of the specific mobilizer or body model and can be performed
    // by the BodyNode.

#endif
  }

  void PrintTopology() const {
    std::cout << "BodyNode id: " << topology_.id << std::endl;
    std::cout << "  Level: " << topology_.level << std::endl;
    std::cout << "  Mobilizer: " << topology_.mobilizer << std::endl;
    std::cout << "  Body: " << topology_.body << std::endl;
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

 private:
  BodyNodeTopology topology_;
  // Pointers for fast access.
  const Body<T>* body_{nullptr};
  const Mobilizer<T>* mobilizer_{nullptr};

  // Helper methods to extract entries from the context.

  // Get from the position kinematics cache a constant reference to the pose
  // X_PF of the "fixed" frame F as measured and expressed in the inboard
  // (parent) body frame P.
  const Isometry3<T>& get_X_PF(const PositionKinematicsCache<T>* pc) const {
    const auto& pool = pc->get_X_BF_pool();
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

  Isometry3<T>& get_mutable_X_PB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_PB(topology_.id);
  }

  Isometry3<T>& get_mutable_X_WB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_WB(topology_.id);
  }

  void CalcAcrossMobilizerBodyPoses(
      const MultibodyTreeContext<T>& context) const {
    PositionKinematicsCache<T>* pc = context.get_mutable_position_kinematics();

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
    // - X_WB(q(W:B), qf_P, qr_B, qf_B)
    Isometry3<T>& X_PB = get_mutable_X_PB(pc);
    Isometry3<T>& X_WB = get_mutable_X_WB(pc);

    // TODO(amcastro-tri): Consider logic for the common case B = M.
    // In that case X_FB = X_FM as suggested by setting X_MB = Id.
    const Isometry3<T> X_FB = X_FM * X_MB;

    if (topology_.F_equals_P) {
      X_PB = X_FB;
    } else {
      const Isometry3<T>& X_PF = get_X_PF(pc);
      X_PB = X_PF * X_FB;
    }

    X_WB = X_WP * X_PB;
  }
};

}  // namespace multibody
}  // namespace drake
