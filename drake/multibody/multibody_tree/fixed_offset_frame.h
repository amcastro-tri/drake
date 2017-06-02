#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template <class T> class BodyFrame;
template <class T> class MultibodyTree;
template <class T> class RigidBody;

/// %FixedOffsetFrame represents a material frame F whose pose is fixed with
/// respect to a _parent_ material frame P. The pose offset is given by a
/// spatial transform `X_PF`, which is constant after construction. For
/// instance, we could rigidly attach a frame F to move with a rigid body B at a
/// fixed pose `X_BF`, where B is the BodyFrame associated with body B.
/// Thus, the World frame pose `X_WF` of a %FixedOffsetFrame F depends only on
/// the World frame pose `X_WP` of its parent P, and the constant pose `X_PF`,
/// with `X_WF=X_WP*X_PF`.
///
/// For more information about spatial transforms, see
/// @ref multibody_spatial_pose. <!-- http://drake.mit.edu/doxygen_cxx/
///                                   group__multibody__spatial__pose.html -->
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class FixedOffsetFrame : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedOffsetFrame)

  /// Creates a material Frame F whose pose is fixed with respect to its
  /// parent material Frame P. The pose is given by a spatial transform `X_PF`;
  /// see class documentation for more information.
  ///
  /// @param[in] P The frame to which this frame is attached with a fixed pose.
  /// @param[in] X_PF The transform giving the pose of F in P.

  // TODO(amcastro-tri): allow to chain multiple frames of type
  // FixedOffsetFrame. An approach would consist on holding a reference to the
  // parent frame of the root FixedOffsetFrame of the chain and X_PF_ would
  // then be set to (at construction) to the pose of this frame on that parent
  // frame.
  FixedOffsetFrame(const BodyFrame<T>& P, const Isometry3<T>& X_PF);

  /// Creates a material Frame F whose pose is fixed with respect to the
  /// BodyFrame B of the given Body, which serves as F's parent frame.
  /// The pose is given by a spatial transform `X_BF`; see class documentation
  /// for more information.
  ///
  /// @param[in] bodyB The body whose BodyFrame B is to be F's parent frame.
  /// @param[in] X_BF  The transform giving the pose of F in B.
  FixedOffsetFrame(const Body<T>& bodyB, const Isometry3<T>& X_BF);

  /// Returns the pose `X_BF` of `this` frame F as measured and expressed in
  /// frame B of the body associated with this frame.
  /// @sa CalcBodyPoseInThisFrame() which returns the inverse
  /// transformation `X_FB`.
  Isometry3<T> CalcPoseInBodyFrame(
      const MultibodyTreeContext<T>& context) const final {
    // X_BF = X_BP * X_PF
    parent_frame_.CalcPoseInBodyFrame(context) * X_PF_;
  }

  /// Returns the pose `X_FB` of the body B associated with this frame F,
  /// measured in this frame F.
  /// @sa CalcBodyPoseInOtherFrame()
  Isometry3<T> CalcBodyPoseInThisFrame(
      const MultibodyTreeContext<T>& context) const final {
    return parent_frame_.CalcBodyPoseInOtherFrame(context, X_FP_);
  }

  /// Returns the pose `X_QB` of the body B associated with this frame F
  /// measured in a frame Q, given the pose `X_QF` of this frame F measured
  /// in Q.
  /// @sa CalcBodyPoseInThisFrame() to compute the pose of the body associated
  /// with this frame as measured in this frame.
  Isometry3<T> CalcBodyPoseInOtherFrame(
      const MultibodyTreeContext<T>& context,
      const Isometry3<T>& X_QF) const final {
    // This method computes: X_QB = X_QP * X_PB
    // where P is this frame's parent frame
    return parent_frame_.CalcBodyPoseInOtherFrame(context, X_QF * X_FP_);
  }

  /// Given the offset pose `X_FQ` of a frame Q measured in this frame F,
  /// compute the pose of frame Q measured and expressed in the frame B of
  /// the body to which this frame is attached.
  Isometry3<T> CalcOffsetPoseInBody(
      const MultibodyTreeContext<T>& context,
      const Isometry3<T>& X_FQ) const final {
    return parent_frame_.CalcOffsetPoseInBody(context, X_PF_ * X_FQ);
  }

 private:
  // The frame to which this frame is attached.
  const Frame<T>& parent_frame_;

  // Spatial transform giving the fixed pose of this frame F measured in the
  // parent frame P.
  const Isometry3<T> X_PF_;

  // Spatial transform giving the fixed pose of the parent frame P measured in
  // this frame F.
  const Isometry3<T> X_FP_;
};

}  // namespace multibody
}  // namespace drake
