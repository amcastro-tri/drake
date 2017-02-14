#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer_impl.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

// Forward declarations.
template <typename T> class MultibodyTree;

namespace drake {
namespace multibody {

template <typename T>
class RevoluteMobilizer : public MobilizerImpl<T, 1, 1> {
  typedef MobilizerImpl<T, 1, 1> MobilizerBase;
  using MobilizerBase::nq;
  using MobilizerBase::nv;
  using typename MobilizerBase::HMatrix;
  using MobilizerBase::get_id;
 public:
  static RevoluteMobilizer<T>& Create(
      MultibodyTree<T>* tree,
      const MaterialFrame<T>& inboard_frame,
      const MaterialFrame<T>& outboard_frame, const Vector3<double> axis_F);

  /// Sets the state represented in @p context so that the generalized
  /// coordinate representing the rotation angle equals @p angle.
  /// @param[in] context The context of the MultibodyTree this mobilizers
  /// belongs to.
  /// @param[in] angle The desired angle in radians.
  /// @returns a reference to `this` mobilizer.
  const RevoluteMobilizer<T>& set_angle(
      MultibodyTreeContext<T>* context, const T& angle) const;

  const RevoluteMobilizer<T>& set_angular_velocity(
      MultibodyTreeContext<T>* context, const T& angular_velocity) const;

  void CalcAcrossMobilizerTransform(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const final;

  void CalcAcrossMobilizerVelocityJacobian(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const final;

  /// @param[in] context The state of the MultibodyTree.
  /// @param[out] qdot The time derivative of the generalized coordinates for
  ///                  this mobilizer.
  void CalcQDot(const MultibodyTreeContext<T>& context,
                Eigen::Ref<VectorX<T>> qdot) const final;

 private:
  // Creates a revolute joint with axis_F expressed in the inboard frame F.
  RevoluteMobilizer(const MaterialFrame<T>& inboard_frame,
                    const MaterialFrame<T>& outboard_frame,
                    const Vector3<double> axis_F) :
      MobilizerBase(inboard_frame, outboard_frame), axis_F_(axis_F) {}

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_F_;
};

}  // namespace multibody
}  // namespace drake
