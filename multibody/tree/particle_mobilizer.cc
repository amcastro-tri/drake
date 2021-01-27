#include "drake/multibody/tree/particle_mobilizer.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
Vector3<T> ParticleMobilizer<T>::get_position(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q;
}

template <typename T>
const ParticleMobilizer<T>& ParticleMobilizer<T>::set_position(
    systems::Context<T>* context,
    const Eigen::Ref<const Vector3<T>>& translations) const {
  auto q = this->get_mutable_positions(&*context);
  DRAKE_ASSERT(q.size() == kNq);
  q = translations;
  return *this;
}

template <typename T>
Vector3<T> ParticleMobilizer<T>::get_translational_velocity(
    const systems::Context<T>& context) const {
  const auto& v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v;
}

template <typename T>
const ParticleMobilizer<T>& ParticleMobilizer<T>::set_translational_velocity(
    systems::Context<T>* context,
    const Eigen::Ref<const Vector3<T>>& v_FM_F) const {
  auto v = this->get_mutable_velocities(&*context);
  DRAKE_ASSERT(v.size() == kNv);
  v = v_FM_F;
  return *this;
}

template <typename T>
math::RigidTransform<T> ParticleMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  Vector3<T> X_FM_translation;
  X_FM_translation = q;
  return math::RigidTransform<T>(math::RotationMatrix<T>::Identity(),
                                 X_FM_translation);
}

template <typename T>
SpatialVelocity<T> ParticleMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  Vector6<T> V_FM_vector;
  V_FM_vector << 0.0, 0.0, 0.0, v;
  return SpatialVelocity<T>(V_FM_vector);
}

template <typename T>
SpatialAcceleration<T>
ParticleMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  Vector6<T> A_FM_vector;
  A_FM_vector << 0.0, 0.0, 0.0, vdot;
  return SpatialAcceleration<T>(A_FM_vector);
}

template <typename T>
void ParticleMobilizer<T>::ProjectSpatialForce(const systems::Context<T>&,
                                             const SpatialForce<T>& F_Mo_F,
                                             Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  tau = F_Mo_F.translational();
}

template <typename T>
void ParticleMobilizer<T>::DoCalcNMatrix(const systems::Context<T>&,
                                       EigenPtr<MatrixX<T>> N) const {
  *N = Matrix3<T>::Identity();
}

template <typename T>
void ParticleMobilizer<T>::DoCalcNplusMatrix(const systems::Context<T>&,
                                           EigenPtr<MatrixX<T>> Nplus) const {
  *Nplus = Matrix3<T>::Identity();
}

template <typename T>
void ParticleMobilizer<T>::MapVelocityToQDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
  *qdot = v;
}

template <typename T>
void ParticleMobilizer<T>::MapQDotToVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
  *v = qdot;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
ParticleMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<ParticleMobilizer<ToScalar>>(inboard_frame_clone,
                                                     outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>> ParticleMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> ParticleMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
ParticleMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ParticleMobilizer)
