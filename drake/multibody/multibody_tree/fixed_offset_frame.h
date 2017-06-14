#pragma once

#include "drake/common/eigen_autodiff_types.h"
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
  FixedOffsetFrame(const Frame<T>& P, const Isometry3<double>& X_PF);

  /// Creates a material Frame F whose pose is fixed with respect to the
  /// BodyFrame B of the given Body, which serves as F's parent frame.
  /// The pose is given by a spatial transform `X_BF`; see class documentation
  /// for more information.
  ///
  /// @param[in] bodyB The body whose BodyFrame B is to be F's parent frame.
  /// @param[in] X_BF  The transform giving the pose of F in B.
  FixedOffsetFrame(const Body<T>& bodyB, const Isometry3<double>& X_BF);

  /// Returns the pose `X_FB` of the body B associated with this frame F,
  /// measured in this frame F.
  /// @sa CalcBodyPoseInOtherFrame()
  Isometry3<T> CalcBodyPoseInThisFrame(
      const systems::Context<T>& context) const final {
    return parent_frame_.CalcBodyPoseInOtherFrame(context,
                                                  X_PF_.cast<T>().inverse());
  }

  /// Given the offset pose `X_FQ` of a frame Q measured in this frame F,
  /// compute the pose of frame Q measured and expressed in the frame B of
  /// the body to which this frame is attached.
  Isometry3<T> CalcOffsetPoseInBody(
      const systems::Context<T>& context,
      const Isometry3<T>& X_FQ) const final {
    return parent_frame_.CalcOffsetPoseInBody(context,
                                              X_PF_.cast<T>() * X_FQ);
  }

 protected:
  /// @pre The parent frame to this frame already has a clone in `tree_clone`.
  std::unique_ptr<Frame<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final {
    return TemplatedDoCloneToScalar(tree_clone);
  }

  /// @pre The parent frame to this frame already has a clone in `tree_clone`.
  std::unique_ptr<Frame<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final {
    return TemplatedDoCloneToScalar(tree_clone);
  }

 private:
  template <typename ToScalar>
  std::unique_ptr<Frame<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // The frame to which this frame is attached.
  const Frame<T>& parent_frame_;

  // Spatial transform giving the fixed pose of this frame F measured in the
  // parent frame P.
  const Isometry3<double> X_PF_;
};

}  // namespace multibody
}  // namespace drake
