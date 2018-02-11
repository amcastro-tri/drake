#include "drake/multibody/multibody_tree/test/floating_body_plant.h"

#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

template<typename T>
FloatingBodyPlant<T>::FloatingBodyPlant(double mass, double I, double J, double g) :
    mass_(mass), I_(I), J_(J), g_(g) {
  BuildMultibodyTreeModel();
  DRAKE_DEMAND(model_.get_num_positions() == 7);
  DRAKE_DEMAND(model_.get_num_velocities() == 6);
  DRAKE_DEMAND(model_.get_num_states() == 13);
  this->DeclareContinuousState(
      model_.get_num_positions(),
      model_.get_num_velocities(), 0 /* num_z */);
}

template<typename T>
template<typename U>
FloatingBodyPlant<T>::FloatingBodyPlant(
    const FloatingBodyPlant<U> &other) :
    FloatingBodyPlant<T>(other.mass_, other.I_, other.J_) {}

template<typename T>
void FloatingBodyPlant<T>::BuildMultibodyTreeModel() {
  UnitInertia<double> G_Bcm =
      UnitInertia<double>::AxiallySymmetric(J_, I_, Vector3<double>::UnitZ());
  SpatialInertia<double> M_Bcm(mass_, Vector3<double>::Zero(), G_Bcm);

  body_ = &model_.template AddBody<RigidBody>(M_Bcm);

  mobilizer_ =
      &model_.template AddMobilizer<QuaternionFloatingMobilizer>(
          model_.get_world_frame(), body_->get_body_frame());

  model_.template AddForceElement<UniformGravityFieldElement>(
      -g_ * Vector3<double>::UnitZ());

  model_.Finalize();
}

template<typename T>
Vector3<double>
FloatingBodyPlant<T>::get_default_initial_angular_velocity() const {
  return Vector3d::UnitX() + Vector3d::UnitY() + Vector3d::UnitZ();
}

template<typename T>
Vector3<double>
FloatingBodyPlant<T>::get_default_initial_translational_velocity() const {
  return Vector3d::UnitX();
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
FloatingBodyPlant<T>::DoMakeLeafContext() const {
  return model_.CreateDefaultContext();
}

template<typename T>
void FloatingBodyPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T> &context,
    systems::ContinuousState<T> *derivatives) const {
  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();

  const int nq = model_.get_num_positions();
  const int nv = model_.get_num_velocities();

  // Allocate workspace. We might want to cache these to avoid allocations.
  // Mass matrix.
  MatrixX<T> M(nv, nv);
  // Forces.
  MultibodyForces<T> forces(model_);
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(model_.get_num_bodies());
  // Generalized accelerations.
  VectorX<T> vdot = VectorX<T>::Zero(nv);

  model_.CalcMassMatrixViaInverseDynamics(context, &M);

  // Check if M is symmetric.
  const T err_sym = (M - M.transpose()).norm();
  DRAKE_DEMAND(err_sym < 10 * std::numeric_limits<double>::epsilon());


  PositionKinematicsCache<T> pc(model_.get_topology());
  VelocityKinematicsCache<T> vc(model_.get_topology());
  model_.CalcForceElementsContribution(context, pc, vc, &forces);

  // With vdot = 0, this computes:
  //   tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces.mutable_body_forces();
  VectorX<T>& tau_array = forces.mutable_generalized_forces();
  model_.CalcInverseDynamics(
      context, pc, vc, vdot,
      F_BBo_W_array, tau_array,
      &A_WB_array,
      &F_BBo_W_array, /* Notice these arrays gets overwritten on output. */
      &tau_array);

  vdot = M.ldlt().solve(-tau_array);

  auto v = x.bottomRows(nv);
  VectorX<T> xdot(model_.get_num_states());
  VectorX<T> qdot(nq);
  model_.MapVelocityToQDot(context, v, &qdot);
  xdot << qdot, vdot;
  derivatives->SetFromVector(xdot);
}

template<typename T>
void FloatingBodyPlant<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    systems::VectorBase<T>* generalized_velocity) const {
  const int nq = model_.get_num_positions();
  const int nv = model_.get_num_velocities();

  DRAKE_ASSERT(qdot.size() == nq);
  DRAKE_DEMAND(generalized_velocity != nullptr);
  DRAKE_DEMAND(generalized_velocity->size() == nv);

  VectorX<T> v(nv);
  model_.MapQDotToVelocity(context, qdot, &v);
  generalized_velocity->SetFromVector(v);
}

template<typename T>
void FloatingBodyPlant<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    systems::VectorBase<T>* positions_derivative) const {
  const int nq = model_.get_num_positions();
  const int nv = model_.get_num_velocities();

  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_DEMAND(positions_derivative != nullptr);
  DRAKE_DEMAND(positions_derivative->size() == nq);

  VectorX<T> qdot(nq);
  model_.MapVelocityToQDot(context, generalized_velocity, &qdot);
  positions_derivative->SetFromVector(qdot);
}

template<typename T>
void FloatingBodyPlant<T>::SetDefaultState(
    const systems::Context<T>& context, systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  model_.SetDefaultState(context, state);
  mobilizer_->set_angular_velocity(
      context, get_default_initial_angular_velocity(), state);
  mobilizer_->set_translational_velocity(
      context, get_default_initial_translational_velocity(), state);
}

template<typename T>
Vector3<T> FloatingBodyPlant<T>::get_angular_velocity(
    const systems::Context<T>& context) const {
  return mobilizer_->get_angular_velocity(context);
}

template<typename T>
void FloatingBodyPlant<T>::set_angular_velocity(
    systems::Context<T>* context, const Vector3<T>& w_WB) const {
  mobilizer_->set_angular_velocity(context, w_WB);
}

template<typename T>
Isometry3<T> FloatingBodyPlant<T>::CalcPoseInWorldFrame(
    const systems::Context<T>& context) const {
  PositionKinematicsCache<T> pc(model_.get_topology());
  model_.CalcPositionKinematicsCache(context, &pc);
  return pc.get_X_WB(body_->get_node_index());
}

template<typename T>
SpatialVelocity<T> FloatingBodyPlant<T>::CalcSpatialVelocityInWorldFrame(
    const systems::Context<T>& context) const {
  PositionKinematicsCache<T> pc(model_.get_topology());
  model_.CalcPositionKinematicsCache(context, &pc);
  VelocityKinematicsCache<T> vc(model_.get_topology());
  model_.CalcVelocityKinematicsCache(context, pc, &vc);
  return vc.get_V_WB(body_->get_node_index());
}

}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::multibody_tree::test::FloatingBodyPlant)
