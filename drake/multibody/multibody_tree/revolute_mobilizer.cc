#include "drake/multibody/multibody_tree/revolute_mobilizer.h"

#include "drake/multibody/multibody_tree/mobilizer_impl.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

template <typename T>
RevoluteMobilizer<T>& RevoluteMobilizer<T>::set_angle(
    MultibodyTreeContext<T>* context, const T& angle) {
  Vector<T, nq>& q = this->get_mutable_positions(context);
  q[0] = angle;
  return *this;
}

template <typename T>
RevoluteMobilizer<T>& RevoluteMobilizer<T>::set_angular_velocity(
    MultibodyTreeContext<T>* context, const T& angular_velocity) {
  Vector<T, nq>& q = this->get_mutable_velocities(context);
  q[0] = angular_velocity;
  return *this;
}

template <typename T>
void RevoluteMobilizer<T>::CalcAcrossMobilizerTransform(
    const MobilizerContext<T>& context,
    MobilizerPositionKinematics<T>* pc) const {
  const Vector<T, nq>& q = context.template get_positions<num_positions>();
  Isometry3<T>& X_FM = pc->get_mutable_X_FM();
  X_FM = Isometry3<T>::Identity();
  X_FM.linear() = Eigen::AngleAxis<T>(q[0], axis_F_).toRotationMatrix();
}

template <typename T>
void RevoluteMobilizer<T>::CalcAcrossMobilizerVelocityJacobian(
    const MobilizerContext<T>& context,
    MobilizerPositionKinematics<T>* pc) const {
  HMatrix& H_FM = pc->template get_mutable_H_FM<num_velocities>();
  H_FM.col(0).angular() = axis_F_;
  H_FM.col(0).linear() = Vector3<T>::Zero();
}

#if 0
template <typename T>
void RevoluteMobilizer<T>::CalcAcrossJointVelocityJacobian(
    const Eigen::Ref<const VectorX<T>>& q, Eigen::Ref<MatrixX<T>> Ht) const {
  // auto& V = SpatialVector::mutable_view(Ht.col(0));
  // V.angular() = axis_F_;
  // V.linear() = Vector3<T>::Zero();
  Ht.template head<3>() = axis_F_;  // Angular component.
  Ht.template tail<3>() = Vector3<T>::Zero();  // Linear component.
}
#endif

// Explicitly instantiates on the most common scalar types.
template class RevoluteMobilizer<double>;

}  // namespace multibody
}  // namespace drake
