#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <typename T>
class WorldFrame final : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WorldFrame)

  /// Constructs a world frame associated with the world body frame.
  WorldFrame(const BodyFrame<T>& world_body_frame) :
      Frame<T>(nullptr, &world_body_frame.get_body()),
      implementation_(&world_body_frame) {}

  /// @name Advanced level methods.
  /// @{
  /// Returns the pose `X_BF` of `this` frame F in the body frame B associated
  /// with this frame.
  /// In particular, if `this` **is** the body frame B, this method directly
  /// returns the identity transformation.
  Isometry3<T> CalcPoseInBodyFrame(
      const systems::Context<T>& context) const override {
    DRAKE_DEMAND(implementation_ != nullptr);
    return implementation_->CalcPoseInBodyFrame(context);
  }

  /// Given the offset pose `X_FQ` of a frame Q in `this` frame F, this method
  /// computes the pose `X_BQ` of frame Q in the body frame B to which this
  /// frame is attached.
  /// In other words, if the pose of `this` frame F in the body frame B is
  /// `X_BF`, this method computes the pose `X_BQ` of frame Q in the body frame
  /// B as `X_BQ = X_BF * X_FQ`.
  /// In particular, if `this` **is**` the body frame B, i.e. `X_BF` is the
  /// identity transformation, this method directly returns `X_FQ`.
  /// Specific frame subclasses can override this method to provide faster
  /// implementations if needed.
  Isometry3<T> CalcOffsetPoseInBody(
      const systems::Context<T>& context,
      const Isometry3<T>& X_FQ) const override {
    DRAKE_DEMAND(implementation_ != nullptr);
    return implementation_->CalcOffsetPoseInBody(context, X_FQ);
  }
  /// @}

  // Hide the following section for internal methods from Doxygen.
  // These methods are intended for internal use only.
#ifndef DRAKE_DOXYGEN_CXX
  // (Internal) Get the underlying implementation (a BodyFrame) for this frame.
  const BodyFrame<T>& get_implementation() const {
    DRAKE_DEMAND(implementation_ != nullptr);
    return *implementation_;
  }
#endif

 protected:
  // Frame<T>::DoCloneToScalar() overrides.
  std::unique_ptr<Frame<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Frame<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  // Make WorldFrame templated on any other scalar type a friend of
  // WorldFrame<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private methods from BodyFrame<T>.
  template <typename> friend class WorldFrame;

  // Helper method to make a clone templated on any other scalar type.
  // This method holds the common implementation for the different overrides to
  // DoCloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<Frame<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // The internal body frame this link frame represents.
  const BodyFrame<T>* implementation_{nullptr};
};

}  // namespace multibody
}  // namespace drake
