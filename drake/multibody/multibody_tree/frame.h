#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_member.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <typename T>
class Frame : public MultibodyTreeMember<Frame<T>, FrameIndex> {
 public:
  //virtual const Isommetry3<T>& get_pose_in_world(
  //   const MultibodyTreeContext<T>& context) const = 0;
};

template <typename T>
class BodyFrame : public Frame<T> {
 public:
  BodyFrame(const Body<T>& body);

  const Body<T>& get_body() const;

  BodyIndex get_body_id() const;

  /// @returns X_BF the pose of this frame `F` in the frame `B` of the body to
  /// which this frame rigidly attaches to.
  //const Isometry3<T>& GetPoseInBodyFrame(
  //    const MultibodyTreeContext<T>& context) const {
  //  return get_body().GetFramePoseInBodyFrame(*this, context);
  //}

  // const Isommetry3<T>& get_pose_in_world(
  //   const MultibodyTreeContext<T>& context) const override {
  //   Body<T>& body = get_parent_tree().get_body(body_id);
  //   return body.get_frame_pose_in_world(*this, context);
  // }
 private:
  BodyIndex body_id_{BodyIndex::Invalid()};
};

template <typename T>
class RigidBodyFrame : public BodyFrame<T> {
 public:
  RigidBodyFrame(const Body<T>& body, const Isometry3<T>& X_BF);

  //const Isometry3<T>& GetPoseInBodyFrame(
  //    const MultibodyTreeContext<T>& context) const {
  //  return X_BF_;
  //}

  // const Isommetry3<T>& get_pose_in_world(
  //   const MultibodyTreeContext<T>& context) const override {
  //   Body<T>& body = get_parent_tree().get_body(body_id);
  //   return body.get_frame_pose_in_world(*this, context);
  // }

 private:
  Isometry3<T> X_BF_;
};

}  // namespace multibody
}  // namespace drake
