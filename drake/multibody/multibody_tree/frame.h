#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

#include <iostream>

namespace drake {
namespace multibody {

// Forward declarations.
template <class T> class Body;
template <class T> class MultibodyTree;
template <class T> class RigidBody;

template <typename T>
class Frame : public MultibodyTreeElement<Frame<T>, FrameIndex> {};

template <typename T>
class MaterialFrame : public Frame<T> {
 public:
  BodyIndex get_body_id() const { return topology_.body_id;}

  const MaterialFrameTopology get_topology() const { return topology_;}

  /// Sets the topological information for this mobilizer.
  /// This is an implementation detail. User code should never call it.
  void SetTopology(const MaterialFrameTopology& topology) {
    topology_ = topology;
  }

  /// Gives frame the opportunity to set default entries in the context
  /// tipically in the form of cache entries. This allows to precompute cache
  /// entries that do not change in time at initialization.
  /// Defaults to a no-op.
  virtual void SetDefaults(MultibodyTreeContext<T>* context) {}

  void PrintTopology() const {
    std::cout << "Frame id: " << topology_.id << std::endl;
    std::cout << "Body id: " << topology_.body_id << std::endl;
    std::cout << "Local body id: " << topology_.local_id << std::endl;
    std::cout << "BodyNode id: " << topology_.body_node << std::endl;
    std::cout << "X_BF_index: " << topology_.X_BF_index << std::endl;
  }

 protected:
  /// @param[in] Local frame id in the body referenced by @p body_id.
  MaterialFrame(const Body<T>& body);

  void set_id(FrameIndex id) override {
    MultibodyTreeElement<Frame<T>, FrameIndex>::set_id(id);
    topology_.id = id;
  }

 private:
  MaterialFrameTopology topology_;
};

template <typename T>
class BodyFrame : public MaterialFrame<T> {
 public:
  /// Creates a new BodyFrame and adds it to the MultibodyTree @p tree.
  /// The MultibodyTree @param tree takes ownership of the frame.
  static BodyFrame<T>& Create(MultibodyTree<T>* tree, const Body<T>& body);

 private:
  BodyFrame(const Body<T>& body);
};

/// This class represents a frame `F` with pose `X_BF` measured and expressed in
/// the body frame `B` of a rigid body.
template <typename T>
class RigidBodyFrame : public MaterialFrame<T> {
 public:
  static RigidBodyFrame<T>& Create(
      MultibodyTree<T>* tree,
      const RigidBody<T>& body, const Isometry3<T>& X_BF);

  void SetDefaults(MultibodyTreeContext<T>* context) final;

 private:
  RigidBodyFrame(const RigidBody<T>& B, const Isometry3<T>& X_BF);

  Isometry3<T> X_BF_;
};

#if 0
/// This class represents a frame `F` with a fixed offset `X_MF` to a material
/// frame `M`.
/// This material frame `M` could either be:
///   - A body frame `B` case in which `F` will move rigidly with `B`.
///   - A SoftMaterialFrame `M` on a body `B` case in which the pose of `F` in
///     the frame of the body will be determined by the
///     flexible degrees of freedom of that body.
template <typename T>
class FixedOffsetFrame : public MaterialFrame<T> {
 public:
  FixedOffsetFrame(const MaterialFrame<T>& M, const Isometry3<T>& X_MF);

 private:
  Isometry3<T> X_MF_;
  // Id of the MaterialFrame this frame moves with.
  FrameIndex material_frame_id_;
};
#endif

#if 0
/// This class represents a frame `M` attached to a material point on a
/// soft body `B`. The pose `X_BM(qf_B)` of frame `M` measured and expressed in
/// `B` will in genearal be a function of the flexible degrees of freedom `qf_B`
/// of body `B`.
template <typename T>
class SoftBodyFrame : public MaterialFrame<T> {
 public:
  SoftBodyFrame(const SoftBody<T>& body) : MaterialFrame(body.get_id()) {}

  /// @returns the local identifier of this frame in the parent SoftBody.
  int get_local_id() const { return local_id_;}

  /// Sets the local identifier of this frame in the parent SoftBody.
  /// Users SHOULD NOT call this method. It is used internally for quick
  /// indexing of each SoftBody's frames.
  void set_local_id(int local_id) const { local_id_ = local_id;}

 private:
  int local_id_;  // Id of this frame in the parent SoftBody.
};
#endif

}  // namespace multibody
}  // namespace drake
