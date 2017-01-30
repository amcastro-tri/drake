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
  using MobilizerBase::HMatrix;
 public:
  /// Creates a revolute joint with axis_F expressed in the inboard frame F.
  RevoluteMobilizer(const BodyFrame<T>& inboard_frame,
                    const BodyFrame<T>& outboard_frame,
                    const Vector3<double> axis_F) :
      MobilizerBase(inboard_frame, outboard_frame), axis_F_(axis_F) {}

  Isometry3<T> CalcAcrossMobilizerTransform(
      const Eigen::Ref<const VectorX<T>>& q) const final;

 private:
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = 1, nv = 1};

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_F_;
};

}  // namespace multibody
}  // namespace drake
