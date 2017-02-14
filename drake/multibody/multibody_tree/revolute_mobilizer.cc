#include "drake/multibody/multibody_tree/revolute_mobilizer.h"

#include "drake/multibody/multibody_tree/mobilizer_impl.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

template <typename T>
RevoluteMobilizer<T>& RevoluteMobilizer<T>::Create(
    MultibodyTree<T>* tree,
    const MaterialFrame<T>& inboard_frame,
    const MaterialFrame<T>& outboard_frame, const Vector3<double> axis_F) {
  // Notice that here we cannot use std::make_unique since constructors are made
  // private to avoid users creating mobilizers by other means other than
  // calling Create().
  RevoluteMobilizer<T>* mobilizer =
      new RevoluteMobilizer<T>(inboard_frame, outboard_frame, axis_F);
  // tree takes ownership.
  MobilizerIndex mobilizer_id =
      tree->AddMobilizer(std::unique_ptr<Mobilizer<T>>(mobilizer));
  mobilizer->set_parent_tree(tree);
  mobilizer->set_id(mobilizer_id);

  return *mobilizer;
}

template <typename T>
const RevoluteMobilizer<T>& RevoluteMobilizer<T>::set_angle(
    MultibodyTreeContext<T>* context, const T& angle) const {
  Vector<T, nq>& q = this->get_mutable_positions(context);
  q[0] = angle;
  return *this;
}

template <typename T>
const RevoluteMobilizer<T>& RevoluteMobilizer<T>::set_angular_velocity(
    MultibodyTreeContext<T>* context, const T& angular_velocity) const {
  Vector<T, nq>& v = this->get_mutable_velocities(context);
  v[0] = angular_velocity;
  return *this;
}

template <typename T>
void RevoluteMobilizer<T>::CalcAcrossMobilizerTransform(
    const MultibodyTreeContext<T>& context,
    PositionKinematicsCache<T>* pc) const {
  const Vector<T, nq>& q = this->get_positions(context);
  Isometry3<T>& X_FM = this->get_mutable_X_FM(pc);
  X_FM = Isometry3<T>::Identity();
  X_FM.linear() = Eigen::AngleAxis<T>(q[0], axis_F_).toRotationMatrix();
}

template <typename T>
void RevoluteMobilizer<T>::CalcAcrossMobilizerVelocityJacobian(
    const MultibodyTreeContext<T>& context,
    PositionKinematicsCache<T>* pc) const {
  HMatrix& H_FM = this->get_mutable_H_FM(pc);
  H_FM.col(0).angular() = axis_F_;
  H_FM.col(0).linear() = Vector3<T>::Zero();
}

template <typename T>
void RevoluteMobilizer<T>::CalcQDot(
    const MultibodyTreeContext<T>& context, Eigen::Ref<VectorX<T>> qdot) const {
  DRAKE_ASSERT(qdot.size() == nq);
  this->VelocityView(qdot) = this->get_velocities(context);
}

// Explicitly instantiates on the most common scalar types.
template class RevoluteMobilizer<double>;

}  // namespace multibody
}  // namespace drake
