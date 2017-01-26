#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/joint_impl.h"

namespace drake {
namespace multibody {

template <typename T>
class RevoluteJoint : public JointImpl<T, 1, 1> {
  typedef JointImpl<T, 1, 1> JointBase;
  using Joint<T>::parent_tree_;
 public:
  /// Creates a revolute joint with axis_F expressed in the inboard frame F.
  RevoluteJoint(const Body<T>& parent_body, const Body<T>& child_body,
                const Eigen::Isometry3d& X_PF, const Eigen::Isometry3d& X_BM,
                const Vector3<double> axis_F) :
      JointBase(parent_body, child_body, X_PF, X_BM), axis_F_(axis_F) {}


  Isometry3<T> CalcAcrossJointTransform(
      const Eigen::Ref<const VectorX<T>>& q) const final;

  void CalcAcrossJointVelocityJacobian(
      const Eigen::Ref<const VectorX<T>>& q, Eigen::Ref<MatrixX<T>> Ht) const;

 private:
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = 7, nv = 6};

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_F_;
};

}  // namespace multibody
}  // namespace drake
