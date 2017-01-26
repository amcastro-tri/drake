#include "drake/multibody/multibody_tree/revolute_joint.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

template <typename T>
Isometry3<T> RevoluteJoint<T>::CalcAcrossJointTransform(
    const Eigen::Ref<const VectorX<T>>& q) const {
  DRAKE_ASSERT(q.size() == this->get_num_qs());
  Isometry3<T> X_FM = Isometry3<T>::Identity();
  X_FM.linear() = Eigen::AngleAxis<T>(q[0], axis_F_).toRotationMatrix();
  return X_FM;
}

template <typename T>
void RevoluteJoint<T>::CalcAcrossJointVelocityJacobian(
    const Eigen::Ref<const VectorX<T>>& q, Eigen::Ref<MatrixX<T>> Ht) const {
  // auto& V = SpatialVector::mutable_view(Ht.col(0));
  // V.angular() = axis_F_;
  // V.linear() = Vector3<T>::Zero();
  Ht.template head<3>() = axis_F_;  // Angular component.
  Ht.template tail<3>() = Vector3<T>::Zero();  // Linear component.
}

// Explicitly instantiates on the most common scalar types.
template class RevoluteJoint<double>;

}  // namespace multibody
}  // namespace drake
