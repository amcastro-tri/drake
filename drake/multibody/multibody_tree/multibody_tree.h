#pragma once

// TODO: consider creating pure virtual interfaces for Body, Joint and
// MultibodyTree all living in a single file as per discussion with Rico.
// That avoids cyclic inclusions and, in principle, offers users (and
// developers) a clear mental picture of the whole thing.

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/body_node.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyTree {
 public:
  /// Creates a MultibodyTree containing only a *world* body (with unique BodyId
  /// equal to 0).
  MultibodyTree();

  /// Takes ownership of @p body and assigns a unique id to it.
  /// @note
  BodyIndex AddBody(std::unique_ptr<Body<T>> body);

  FrameIndex AddMaterialFrame(std::unique_ptr<MaterialFrame<T>> frame);

  MobilizerIndex AddMobilizer(std::unique_ptr<Mobilizer<T>> mobilizer);

  /// Returns the number of bodies in the MultibodyTree including including the
  /// *world* body. Therefore the minimum number of bodies in a MultibodyTree to
  /// which no other bodies have been added, is one.
  int get_num_bodies() const { return static_cast<int>(bodies_.size()); }

  int get_num_frames() const { return static_cast<int>(material_frames_.size()); }

  int get_num_mobilizers() const {
    return static_cast<int>(mobilizers_.size());
  }

  //int get_num_mobilizers() const { return static_cast<int>(mobilizers_.size()); }

  int get_num_levels() const { return static_cast<int>(body_levels_.size()); }

  /// Returns a constant reference to the *world* body.
  const Body<T>& get_world_body() const {
    return *bodies_[0];
  }

  const BodyFrame<T>& get_world_frame() const {
    return *static_cast<const BodyFrame<T>*>(
        material_frames_[bodies_[0]->get_body_frame_id()].get());
  }

  const Body<T>& get_body(BodyIndex body_id) const {
    DRAKE_ASSERT(body_id.is_valid() && body_id < get_num_bodies());
    return *bodies_[body_id];
  }

  Body<T>* get_mutable_body(BodyIndex body_id) const {
    return bodies_[body_id].get();
  }

  const Mobilizer<T>& get_mobilizer(MobilizerIndex mobilizer_id) const {
    DRAKE_ASSERT(
        mobilizer_id.is_valid() && mobilizer_id < get_num_mobilizers());
    return *mobilizers_[mobilizer_id];
  }

  const MaterialFrame<T>& get_material_frame(FrameIndex frame_id) const {
    DRAKE_ASSERT(
        frame_id.is_valid() && frame_id < get_num_frames());
    return *material_frames_[frame_id];
  }

  //const Mobilizer<T>& get_joint(MobilizerIndex joint_id) const;
  //const Body<T>& get_body_inboard_body(BodyIndex body_id) const;

  /// This method must be called after all elements in the tree (joints, bodies,
  /// force elements, constraints) were added and before any computations.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and any other elements connect with each other.
  /// Sizes needed to allocate Context and Cache entries are determined with
  /// this call.
  void Compile();

  const MultibodyTreeTopology& get_topology() const { return topology_;}

  /// Prints the topology of this multibody tree to standard output.
  /// Useful for debugging.
  void PrintTopology() const;

  void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const;

  void UpdateVelocityKinematicsCache(
      const MultibodyTreeContext<T>& context) const;

  /// This method updates the cache entry for composite body inertias which
  /// contains the composite body inertia `R_Bo_W` for each body `B` computed
  /// about this body frame's origing `Bo` and expressed in the world frame `W`.
  /// This method assumes that postion kinematics were already computed by an
  /// invocation to UpdatePositionKinematicsCache() which updates:
  // - Spatial inertia M_Bo_W of body B.
  // - Shift operator phi_PB_W from parent to child body.
  void UpdateCompositeBodyInertiasCache(
      const MultibodyTreeContext<T>& context) const {
    // TODO: check if positions kinematics is updated. Throw if not.
    // Specifically, this method needs from the cache:
    // - Spatial inertia M_Bo_W of body B.
    // - Shift operator phi_PB_W from parent to child body.
    CompositeBodyInertiasCache<T>* cbi_cache = context.get_mutable_cbi_cache();
    CalcCompositeBodyInertias(
        context, cbi_cache->get_mutable_R_Bo_W_pool());
    // TODO: mark cache entry as updated.
  }

  /// Computes the Composite Body Inertia (CBI) associated with each body.
  /// The composite body inertia associated with a body is defined as the
  /// effective spatial inertia of the composite body formed by all bodies
  /// outboard of the mobilizer that endows rigid body motions to that body.
  ///
  /// For an in depth discussion refer to Section 4.1.2 of A. Jain's
  /// book, p. 59.
  ///
  /// The composite body inertia for the i-th body around its frame origin Boi
  /// is computed as:
  ///   Ri(i) = Ii(i) + \sum_{\forall j\in C(i)} Rj(i)
  /// where C(i) is the set of bodies children of body i. In the
  /// equation above it is implicit that the CRB inertia for body j computed
  /// about body-j's origin (Rj(j)) is translated to body-i's origin Boi so that
  /// the summation is valid. This translation transformation is described by
  /// the parallel axis theorem for spatial inertias (Eq. 2.12 in A. Jain's
  /// book, p. 20).
  /// This method is O(N) with N the number of bodies in the tree.
  ///
  /// Computing CRB's about the world's origin might lead to numerical problems
  /// for large offsets from the origin. This might become more noticeable for
  /// instance in car simulations with large offsets of the many smaller
  /// components of a car from the origin.
  /// A better approach is described in Section 4.1.2 of A. Jain's book, p. 59
  /// where the CBI of body k is computed about the body frame origin for
  /// body this body k. All CBI's are expressed in the world frame `W`.
  void CalcCompositeBodyInertias(
      const MultibodyTreeContext<T>& context,
      eigen_aligned_std_vector<SpatialInertia<T>>& cbi_array) const;

  /// This method implements the Newton-Euler inverse dynamics algorithm for
  /// computing the inverset dynamics of this MultibodyTree. This algorithm is
  /// `O(N)` in complexity and involves a base-to-tip recursion to compute
  /// spatial velocities and accelerations and a tip-to-base recursion to
  /// compute generalized forces at each mobilizer.
  /// For a detalied discussion of the algorithm refer to Section 5.3.1 of
  /// Jain (2010), p. 88.
  void InverseDynamics(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot,
      const Eigen::Ref<const VectorX<T>>& external_generalized_forces,
      const std::vector<SpatialVector<T>>& external_body_forces,
      std::vector<SpatialVector<T>>* body_spatial_accelerations,
      Eigen::Ref<VectorX<T>> generalized_forces) const;

#if 0
  /// Computes the positive definite mass matrix of the multibody system in
  /// quasi-coordinates \f$M(q)\f$, defined by
  /// \f$T = \frac{1}{2} v^T M(q) v\f$, where \f$ T\f$ is the kinetic energy.
  ///
  /// The mass matrix also appears in the manipulator equations
  ///  \f[
  ///  M(q) \dot{v} + C(q, v, f_\text{ext}) = B(q) u
  /// \f]
  ///
  /// This method uses The Composite Rigid Body (CRB) method, which being
  /// O(N^2) in computational complexity, is the most efficient approch for
  /// computing the mass matrix explicitly.
  ///
  /// @param[in] context The current state of the system.
  /// @param[out] MassMatrix A pointer to a properly sized matrix to hold the
  ///                        mass matrix \f$ M(q) \f$.
  ///
  /// Note: Calling this method updates, if needed, the cache entry for
  /// composite body inertias.
  void CalcMassMatrix(const MultibodyTreeContext<T>& context,
                            MatrixX<T>* MassMatrix);
#endif

  std::unique_ptr<MultibodyTreeContext<T>> CreateDefaultContext() const {
    auto context = std::make_unique<MultibodyTreeContext<T>>(topology_);
    SetDefaults(context.get());
    return context;
  }

  /// Sets default values in the context including pre-computed cache entries.
  void SetDefaults(MultibodyTreeContext<T>* context) const {
    // Gives materials frames with a constant offset to their bodies the
    // chance to write their value into the cache.
    for (const auto& frame: material_frames_)
      frame->SetDefaults(context);
  }

 private:
  // Invalidates tree topology when it changes.
  // See methods AddBody() and AddJoint().
  void InvalidateTopology() {
    topology_.invalidate();
  }

  void CompileTopology();

  std::vector<std::unique_ptr<MaterialFrame<T>>> material_frames_;
  std::vector<std::unique_ptr<Body<T>>> bodies_;
  std::vector<std::unique_ptr<Mobilizer<T>>> mobilizers_;

  std::vector<std::unique_ptr<BodyNode<T>>> body_nodes_;

  // Topology cache: This is all the information needed regarding the
  // connectivity of the system that allows to perform efficient traversals for
  // multibody algorithms.
  // Maybe place these in a MultibodyTopology struct. In principle that struct
  // is so simple (only composed by indexes) that we could just simply copy it
  // when cloning or transmogrifying a MultibodyTree.

  // for the level-th level in the tree, body_levels_[level] contains the
  // indexes of all bodies in that level. level = 0 refers to the world body.
  std::vector<std::vector<BodyIndex>> body_levels_;

  std::vector<std::vector<BodyNodeIndex>> body_node_levels_;

  // This struct contains all the topology information for this MultibodyTree.
  // When cloning/transmogrifying this struct can be copied right away.
  MultibodyTreeTopology topology_;

  int num_positions_{0};
  int num_velocities_{0};
};

}  // namespace multibody
}  // namespace drake
