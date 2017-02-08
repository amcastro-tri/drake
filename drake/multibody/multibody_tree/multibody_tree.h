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

  const Body<T>& get_body(BodyIndex body_id) const;

  Body<T>* get_mutable_body(BodyIndex body_id) const {
    return bodies_[body_id].get();
  }

  const Mobilizer<T>& get_mobilizer(MobilizerIndex mobilizer_id) const;

  //const Mobilizer<T>& get_joint(MobilizerIndex joint_id) const;
  //const Body<T>& get_body_inboard_body(BodyIndex body_id) const;

  /// This method must be called after all elements in the tree (joints, bodies,
  /// force elements, constraints) were added and before any computations.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and any other elements connect with each other.
  /// Sizes needed to allocate Context and Cache entries are determined with
  /// this call.
  void Compile();

  /// Prints the topology of this multibody tree to standard output.
  /// Useful for debugging.
  void PrintTopology() const;

  void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const;

  void UpdateCompositeBodyInertiasCache(
      const MultibodyTreeContext<T>& context) const {
    // TODO: check if positions kinematics is updated. Throw if not.
    CompositeBodyInertiasCache<T>* cbi_cache = context.get_mutable_cbi_cache();
    CalcCompositeBodyInertias(
        context, cbi_cache->get_mutable_R_Bo_W_pool());
    // TODO: mark cache entry as updated.
  }

  void CalcCompositeBodyInertias(
      const MultibodyTreeContext<T>& context,
      eigen_aligned_std_vector<SpatialInertia<T>>& cbi_array) const;

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
