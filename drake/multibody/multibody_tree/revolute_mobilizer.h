#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer_impl.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

namespace drake {
namespace multibody {

template <typename T>
class RevoluteMobilizer : public MobilizerImpl<T, 1, 1> {
  typedef MobilizerImpl<T, 1, 1> MobilizerBase;
  using typename MobilizerBase::HMatrix;
  using MobilizerBase::num_positions;
  using MobilizerBase::num_velocities;
  using MobilizerBase::get_id;
 public:
  /// Creates a revolute joint with axis_F expressed in the inboard frame F.
  RevoluteMobilizer(const BodyFrame<T>& inboard_frame,
                    const BodyFrame<T>& outboard_frame,
                    const Vector3<double> axis_F) :
      MobilizerBase(inboard_frame, outboard_frame), axis_F_(axis_F) {}

  /// Sets the state represented in @p context so that the generalized
  /// coordinate representing the rotation angle equals @p angle.
  /// @param[in] context The context of the MultibodyTree this mobilizers
  /// belongs to.
  /// @param[in] angle The desired angle in radians.
  /// @returns a reference to `this` mobilizer.
  RevoluteMobilizer<T>& set_angle(MultibodyTreeContext<T>* context,
                                  const T& angle);

  RevoluteMobilizer<T>& set_angular_velocity(MultibodyTreeContext<T>* context,
                                             const T& angular_velocity);

  void CalcAcrossMobilizerTransform(
      const MobilizerContext<T>& context,
      MobilizerPositionKinematics<T>* pc) const final;

  void CalcAcrossMobilizerVelocityJacobian(
      const MobilizerContext<T>& context,
      MobilizerPositionKinematics<T>* pc) const final;

 private:
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = 1, nv = 1};

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_F_;
};

}  // namespace multibody
}  // namespace drake
