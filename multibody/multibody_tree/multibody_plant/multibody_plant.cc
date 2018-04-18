#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;
//#define PRINT_VAR(a) (void) a;
//#define PRINT_VARn(a) (void) a;

namespace drake {
namespace multibody {
namespace multibody_plant {

// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBP_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBP_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)

using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::PenetrationAsPointPair;
using geometry::SourceId;
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::State;

using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyTree;
using drake::multibody::MultibodyTreeContext;
using drake::multibody::PositionKinematicsCache;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::VelocityKinematicsCache;
using systems::BasicVector;
using systems::Context;
using systems::InputPortDescriptor;

template<typename T>
MultibodyPlant<T>::MultibodyPlant(double time_step) :
    systems::LeafSystem<T>(systems::SystemTypeTag<
        drake::multibody::multibody_plant::MultibodyPlant>()),
    time_step_(time_step) {
  model_ = std::make_unique<MultibodyTree<T>>();
}

template<typename T>
template<typename U>
MultibodyPlant<T>::MultibodyPlant(const MultibodyPlant<U>& other) {
  DRAKE_THROW_UNLESS(other.is_finalized());
  model_ = other.model_->template CloneToScalar<T>();
  // Copy of all members related with geometry registration.
  source_id_ = other.source_id_;
  body_index_to_frame_id_ = other.body_index_to_frame_id_;
  geometry_id_to_body_index_ = other.geometry_id_to_body_index_;
  geometry_id_to_visual_index_ = other.geometry_id_to_visual_index_;
  // MultibodyTree::CloneToScalar() already called MultibodyTree::Finalize() on
  // the new MultibodyTree on U. Therefore we only Finalize the plant's
  // internals (and not the MultibodyTree).
  FinalizePlantOnly();
}

template <typename T>
geometry::SourceId MultibodyPlant<T>::RegisterAsSourceForGeometrySystem(
    geometry::GeometrySystem<T>* geometry_system) {
  DRAKE_THROW_UNLESS(geometry_system != nullptr);
  DRAKE_THROW_UNLESS(!geometry_source_is_registered());
  source_id_ = geometry_system->RegisterSource();
  // Save the GS pointer so that on later geometry registrations we can verify
  // the user is making calls on the same GS instance. Only used for that
  // purpose, it gets nullified at Finalize().
  geometry_system_ = geometry_system;
  return source_id_.value();
}

template<typename T>
void MultibodyPlant<T>::RegisterVisualGeometry(
    const Body<T>& body,
    const Isometry3<double>& X_BG, const geometry::Shape& shape,
    geometry::GeometrySystem<T>* geometry_system) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_system != nullptr);
  DRAKE_THROW_UNLESS(geometry_source_is_registered());
  if (geometry_system != geometry_system_) {
    throw std::logic_error(
        "Geometry registration calls must be performed on the SAME instance of "
        "GeometrySystem used on the first call to "
        "RegisterAsSourceForGeometrySystem()");
  }
  GeometryId id;
  // TODO(amcastro-tri): Consider doing this after finalize so that we can
  // register anchored geometry on ANY body welded to the world.
  if (body.index() == world_index()) {
    id = RegisterAnchoredGeometry(X_BG, shape, geometry_system);
  } else {
    id = RegisterGeometry(body, X_BG, shape, geometry_system);
  }
  const int visual_index = geometry_id_to_visual_index_.size();
  geometry_id_to_visual_index_[id] = visual_index;
}

template<typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterCollisionGeometry(
    const Body<T>& body,
    const Isometry3<double>& X_BG, const geometry::Shape& shape,
    const CoulombFriction<double>& coulomb_friction,
    geometry::GeometrySystem<T>* geometry_system) {
  DRAKE_MBP_THROW_IF_FINALIZED();
  DRAKE_THROW_UNLESS(geometry_system != nullptr);
  DRAKE_THROW_UNLESS(geometry_source_is_registered());
  if (geometry_system != geometry_system_) {
    throw std::logic_error(
        "Geometry registration calls must be performed on the SAME instance of "
        "GeometrySystem used on the first call to "
        "RegisterAsSourceForGeometrySystem()");
  }
  GeometryId id;
  // TODO(amcastro-tri): Consider doing this after finalize so that we can
  // register anchored geometry on ANY body welded to the world.
  if (body.index() == world_index()) {
    id = RegisterAnchoredGeometry(X_BG, shape, geometry_system);
  } else {
    id = RegisterGeometry(body, X_BG, shape, geometry_system);
  }
  const int collision_index = geometry_id_to_collision_index_.size();
  geometry_id_to_collision_index_[id] = collision_index;
  DRAKE_ASSERT(
      static_cast<int>(default_coulomb_friction_.size()) == collision_index);
  default_coulomb_friction_.push_back(coulomb_friction);
  return id;
}

template<typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterGeometry(
    const Body<T>& body,
    const Isometry3<double>& X_BG, const geometry::Shape& shape,
    geometry::GeometrySystem<T>* geometry_system) {
  DRAKE_ASSERT(!is_finalized());
  DRAKE_ASSERT(geometry_source_is_registered());
  DRAKE_ASSERT(geometry_system == geometry_system_);
  // If not already done, register a frame for this body.
  if (!body_has_registered_frame(body)) {
    body_index_to_frame_id_[body.index()] =
        geometry_system->RegisterFrame(
            source_id_.value(),
            GeometryFrame(
                body.name(),
                /* Initial pose: Not really used by GS. Will get removed. */
                Isometry3<double>::Identity()));
  }

  // Register geometry in the body frame.
  GeometryId geometry_id = geometry_system->RegisterGeometry(
      source_id_.value(), body_index_to_frame_id_[body.index()],
      std::make_unique<GeometryInstance>(X_BG, shape.Clone()));
  geometry_id_to_body_index_[geometry_id] = body.index();
  return geometry_id;
}

template<typename T>
geometry::GeometryId MultibodyPlant<T>::RegisterAnchoredGeometry(
    const Isometry3<double>& X_WG, const geometry::Shape& shape,
    geometry::GeometrySystem<T>* geometry_system) {
  DRAKE_ASSERT(!is_finalized());
  DRAKE_ASSERT(geometry_source_is_registered());
  DRAKE_ASSERT(geometry_system == geometry_system_);
  GeometryId geometry_id = geometry_system->RegisterAnchoredGeometry(
      source_id_.value(),
      std::make_unique<GeometryInstance>(X_WG, shape.Clone()));
  geometry_id_to_body_index_[geometry_id] = world_index();
  return geometry_id;
}

template<typename T>
void MultibodyPlant<T>::Finalize() {
  model_->Finalize();
  FinalizePlantOnly();
}

template<typename T>
void MultibodyPlant<T>::FinalizePlantOnly() {
  DeclareStateAndPorts();
  // Only declare ports to communicate with a GeometrySystem if the plant is
  // provided with a valid source id.
  if (source_id_) DeclareGeometrySystemPorts();
  DeclareCacheEntries();
  geometry_system_ = nullptr;  // must not be used after Finalize().
  if (get_num_collision_geometries() > 0 &&
      penalty_method_contact_parameters_.time_scale < 0)
    set_penetration_allowance();
  if (get_num_collision_geometries() > 0 &&
      stribeck_model_.stiction_tolerance() < 0)
    set_stiction_tolerance();
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
MultibodyPlant<T>::DoMakeLeafContext() const {
  DRAKE_THROW_UNLESS(is_finalized());
  return std::make_unique<MultibodyTreeContext<T>>(
      model_->get_topology(), is_state_discrete());
}

template<typename T>
void MultibodyPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // No derivatives to compute if state is discrete.
  if (is_state_discrete()) return;

  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const int nv = this->num_velocities();

  // Allocate workspace. We might want to cache these to avoid allocations.
  // Mass matrix.
  MatrixX<T> M(nv, nv);
  // Forces.
  MultibodyForces<T> forces(*model_);
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(model_->num_bodies());
  // Generalized accelerations.
  VectorX<T> vdot = VectorX<T>::Zero(nv);

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Compute forces applied through force elements. This effectively resets
  // the forces to zero and adds in contributions due to force elements.
  model_->CalcForceElementsContribution(context, pc, vc, &forces);

  // If there is any input actuation, add it to the multibody forces.
  if (num_actuators() > 0) {
    Eigen::VectorBlock<const VectorX<T>> u =
        this->EvalEigenVectorInput(context, actuation_port_);
    for (JointActuatorIndex actuator_index(0);
         actuator_index < num_actuators(); ++actuator_index) {
      const JointActuator<T>& actuator =
          model().get_joint_actuator(actuator_index);
      // We only support actuators on single dof joints for now.
      DRAKE_DEMAND(actuator.joint().num_dofs() == 1);
      for (int joint_dof = 0;
           joint_dof < actuator.joint().num_dofs(); ++joint_dof) {
        actuator.AddInOneForce(context, joint_dof, u[actuator_index], &forces);
      }
    }
  }

  model_->CalcMassMatrixViaInverseDynamics(context, &M);

  // WARNING: to reduce memory foot-print, we use the input applied arrays also
  // as output arrays. This means that both the array of applied body forces and
  // the array of applied generalized forces get overwritten on output. This is
  // not important in this case since we don't need their values anymore.
  // Please see the documentation for CalcInverseDynamics() for details.

  // With vdot = 0, this computes:
  //   tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces.mutable_body_forces();
  VectorX<T>& tau_array = forces.mutable_generalized_forces();

  // Compute contact forces on each body by penalty method.
  if (get_num_collision_geometries() > 0) {
    CalcAndAddContactForcesByPenaltyMethod(context, pc, vc, &F_BBo_W_array);
  }

  model_->CalcInverseDynamics(
      context, pc, vc, vdot,
      F_BBo_W_array, tau_array,
      &A_WB_array,
      &F_BBo_W_array, /* Notice these arrays gets overwritten on output. */
      &tau_array);

  vdot = M.ldlt().solve(-tau_array);

  auto v = x.bottomRows(nv);
  VectorX<T> xdot(this->num_multibody_states());
  VectorX<T> qdot(this->num_positions());
  model_->MapVelocityToQDot(context, v, &qdot);
  xdot << qdot, vdot;
  derivatives->SetFromVector(xdot);
}

template<>
template<typename U>
VectorX<U> MultibodyPlant<double>::CalcFischerBurmeisterSolverResidual(
    // state at t0
    const VectorX<double>& v0,
    const MatrixX<double>& M0,
    // External forces (consider making them on <T>)
    const VectorX<double>& tau0,
    // Normal velocity Jacobian (at either tstar or t0)
    const MatrixX<double>& N,
    // Variables
    const VectorX<U>& v, const VectorX<U>& cn) const {
  const double dt = time_step_;  // shorter alias.

  const int nv = v.size();
  const int num_contacts = cn.size();
  const int num_unknowns = nv + num_contacts;

  VectorX<U> R(num_unknowns);

  MatrixX<U> M0_on_U = M0.template cast<U>();
  VectorX<U> v0_on_U = v0.template cast<U>();
  VectorX<U> tau0_on_U = tau0.template cast<U>();

  R.segment(0, nv) = M0_on_U * (v - v0_on_U) / dt + tau0_on_U;

#if 0
  if (std::is_same<U, AutoDiffXd>::value) {
    PRINT_VAR("CalcFischerBurmeisterSolverResidual: AutoDiffXd");
    PRINT_VAR(v(0).derivatives().transpose());
    PRINT_VAR(v(1).derivatives().transpose());
    PRINT_VAR(v(2).derivatives().transpose());
    PRINT_VAR(v(3).derivatives().transpose());
    PRINT_VAR(v(4).derivatives().transpose());
    PRINT_VAR(v(5).derivatives().transpose());

    PRINT_VAR(M0_on_U(0, 0).value());
    PRINT_VAR(M0_on_U(1, 1).value());
    PRINT_VAR(M0_on_U(2, 2).value());
  }
#endif

  if (num_contacts >0 ) {
    MatrixX<U> N_on_U = N.template cast<U>();
    R.segment(0, nv) -= N_on_U.transpose() * cn;

    // Add Fischer-Burmeister terms to residual R.
    for (int icontact = 0; icontact < num_contacts; ++icontact) {
      int iunknown = nv + icontact;
      const U vn = N_on_U.row(icontact) * v;
      R(iunknown) = FischerBurmeisterFunction(vn, cn(icontact));
    }
  }

  return R;
}

template<typename T>
template<typename U>
VectorX<U> MultibodyPlant<T>::CalcFischerBurmeisterSolverResidual(
    // state at t0
    const VectorX<double>& v0,
    const MatrixX<double>& M0,
    // External forces (consider making them on <T>)
    const VectorX<double>& tau0,
    // Normal velocity Jacobian (at either tstar or t0)
    const MatrixX<double>& N,
    // Variables
    const VectorX<U>& v, const VectorX<U>& cn) const {
  DRAKE_ABORT_MSG("T != double not supported");
}

template<>
MatrixX<double> MultibodyPlant<double>::CalcFischerBurmeisterSolverJacobian(
    // state at t0
    const VectorX<double>& v0,
    const MatrixX<double>& M0,
    // External forces (consider making them on <T>)
    const VectorX<double> tau0,
    // Normal velocity Jacobian (at either tstar or t0)
    const MatrixX<double> N,
    const VectorX<double>& v, const VectorX<double>& cn,
    VectorX<double>* R, MatrixX<double>* J) const {
  const int nv = v.size();
  const int num_contacts = cn.size();
  const int num_unknowns = nv + num_contacts;

  VectorX<AutoDiffXd> v_autodiff(nv);
  math::initializeAutoDiff(v, v_autodiff, num_unknowns, 0);

#if 0
  PRINT_VAR(v_autodiff(0).derivatives().transpose());
  PRINT_VAR(v_autodiff(1).derivatives().transpose());
  PRINT_VAR(v_autodiff(2).derivatives().transpose());
  PRINT_VAR(v_autodiff(3).derivatives().transpose());
  PRINT_VAR(v_autodiff(4).derivatives().transpose());
  PRINT_VAR(v_autodiff(5).derivatives().transpose());

  VectorX<AutoDiffXd> cn_autodiff(num_contacts);
  math::initializeAutoDiff(cn, cn_autodiff, num_unknowns, nv);

  PRINT_VAR(num_contacts);
  PRINT_VAR(num_unknowns);
  PRINT_VAR(cn_autodiff.size());
#endif
  
  VectorX<AutoDiffXd> R_autodiff(num_unknowns);

  R_autodiff = CalcFischerBurmeisterSolverResidual(
      v0, M0, tau0, N, v_autodiff, cn_autodiff);

  *R = math::autoDiffToValueMatrix(R_autodiff);
  *J = math::autoDiffToGradientMatrix(R_autodiff);

  PRINT_VAR(R_autodiff(0).value());
  PRINT_VAR(R_autodiff(1).value());
  PRINT_VAR(R_autodiff(2).value());
  PRINT_VAR(R_autodiff(3).value());
  PRINT_VAR(R_autodiff(4).value());
  PRINT_VAR(R_autodiff(5).value());

  PRINT_VAR(R_autodiff(0).derivatives().transpose());
  PRINT_VAR(R_autodiff(1).derivatives().transpose());
  PRINT_VAR(R_autodiff(2).derivatives().transpose());
  PRINT_VAR(R_autodiff(3).derivatives().transpose());
  PRINT_VAR(R_autodiff(4).derivatives().transpose());
  PRINT_VAR(R_autodiff(5).derivatives().transpose());

  PRINT_VARn(*J);

  DRAKE_DEMAND(J->rows() == num_unknowns);
  DRAKE_DEMAND(J->cols() == num_unknowns);

  return *J;
}

template<typename T>
MatrixX<double> MultibodyPlant<T>::CalcFischerBurmeisterSolverJacobian(
    // state at t0
    const VectorX<double>& v0,
    const MatrixX<double>& M0,
    // External forces (consider making them on <T>)
    const VectorX<double> tau0,
    // Normal velocity Jacobian (at either tstar or t0)
    const MatrixX<double> N,
    const VectorX<double>& v, const VectorX<double>& cn,
    VectorX<double>* R, MatrixX<double>* J) const {
  DRAKE_ABORT_MSG("T != double not supported.");
}

template<>
void MultibodyPlant<double>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<double>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& events,
    drake::systems::DiscreteValues<double>* updates) const {
  // If plant state is continuous, no discrete state to update.
  if (!is_state_discrete()) return;

  const double& dt = time_step_;  // shorter alias.

  const int nq = this->num_positions();
  const int nv = this->num_velocities();

  // BIG HACK #1!!! context0 can only be used in MBP/MBT queries!! if used for GS
  // queries or something outside this system scope, it'll cause a crash.
  // Create a copy of context into context0.
  //std::unique_ptr<systems::Context<double>> context0 = this->CreateDefaultContext();
  //DRAKE_DEMAND(context0->get_num_discrete_state_groups() == 1);
  //context0->get_mutable_discrete_state(0).SetFromVector(con);
  //context0->SetTimeStateAndParametersFrom(context);
  Context<double>& context0 = const_cast<Context<double>&>(context);

  // BIG HACK #2!!! get a mutable reference to context so that we can modify it
  // to hold the solution at tstar. That will allow us to perform GS queries.
  Context<double>& context_star = const_cast<Context<double>&>(context);

  // Save the system state as a raw Eigen vector (solution at previous time step).
  auto x0 = context0.get_discrete_state(0).get_value();
  VectorX<double> q0 = x0.topRows(nq);
  VectorX<double> v0 = x0.bottomRows(nv);

  //////////////////////////////////////////////////////////////////////////////
  // WORK WITH CONTEXT0
  //////////////////////////////////////////////////////////////////////////////

  // Allocate workspace. We might want to cache these to avoid allocations.
  // Mass matrix.
  MatrixX<double> M0(nv, nv);
  // Forces.
  MultibodyForces<double> forces(*model_);
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<double>> A_WB_array(model_->num_bodies());
  // Generalized accelerations.
  VectorX<double> vdot = VectorX<double>::Zero(nv);

  const PositionKinematicsCache<double>& pc0 = EvalPositionKinematics(context0);
  const VelocityKinematicsCache<double>& vc0 = EvalVelocityKinematics(context0);

  // Compute forces applied through force elements. This effectively resets
  // the forces to zero and adds in contributions due to force elements.
  model_->CalcForceElementsContribution(context0, pc0, vc0, &forces);

  // If there is any input actuation, add it to the multibody forces.
  if (num_actuators() > 0) {
    Eigen::VectorBlock<const VectorX<double>> u =
        this->EvalEigenVectorInput(context0, actuation_port_);
    for (JointActuatorIndex actuator_index(0);
         actuator_index < num_actuators(); ++actuator_index) {
      const JointActuator<double>& actuator =
          model().get_joint_actuator(actuator_index);
      // We only support actuators on single dof joints for now.
      DRAKE_DEMAND(actuator.joint().num_dofs() == 1);
      for (int joint_dof = 0;
           joint_dof < actuator.joint().num_dofs(); ++joint_dof) {
        actuator.AddInOneForce(context0, joint_dof, u[actuator_index], &forces);
      }
    }
  }

  model_->CalcMassMatrixViaInverseDynamics(context0, &M0);

  // Velocity at next time step.
  VectorX<double> vn(this->num_velocities());

  // With vdot = 0, this computes:
  //   tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  std::vector<SpatialForce<double>>& F_BBo_W_array = forces.mutable_body_forces();
  VectorX<double>& tau0 = forces.mutable_generalized_forces();
  model_->CalcInverseDynamics(
      context0, pc0, vc0, vdot,
      F_BBo_W_array, tau0,
      &A_WB_array,
      &F_BBo_W_array, /* Notice these arrays gets overwritten on output. */
      &tau0);

  //////////////////////////////////////////////////////////////////////////////
  // WORK WITH CONTEXT_STAR
  //////////////////////////////////////////////////////////////////////////////

  // Compute discrete update without contact forces.
  VectorX<double> v_star = vn = v0 + dt * M0.ldlt().solve(-tau0);
  VectorX<double> qdot_star(this->num_positions());
  model_->MapVelocityToQDot(context0, vn, &qdot_star);
  VectorX<double> q_star = q0 + dt * qdot_star;

  // At state star, compute (candidate) contact points.

  //if (get_num_collision_geometries() == 0) return;
  VectorX<double> x_star(this->num_multibody_states());
  x_star << q_star, v_star;
  //std::unique_ptr<systems::LeafContext<double>> context_star = DoMakeLeafContext();
  //std::unique_ptr<systems::Context<double>> context_star = this->CreateDefaultContext();
  DRAKE_DEMAND(context_star.get_num_discrete_state_groups() == 1);
  context_star.get_mutable_discrete_state(0).SetFromVector(x_star);

  std::vector<PenetrationAsPointPair<double>> contact_penetrations_star =
      ComputePenetrations(context_star);
  int num_contacts = contact_penetrations_star.size();

  //////////////////////////////////////////////////////////////////////////////
  // WORK WITH CONTEXT0
  //////////////////////////////////////////////////////////////////////////////
  context0.get_mutable_discrete_state(0).SetFromVector(x0);

  // Compute normal velocities Jacobian at tstar.
  MatrixX<double> Nstar;
  if (num_contacts > 0) {
    // NOTE: The approximation here is to use the state at t0 and the contact
    // penetraions at tstar. Ideally both would be at tc, but then there would
    // be a different tc per contact pair.
    // TODO(amcastro-tri): consider doing something better here. Would it be
    // possible to compute a cheap approximation to tc?
    Nstar = ComputeNormalVelocityJacobianMatrix(
        context0, contact_penetrations_star);
  }

  // Compute constant terms
  //const VectorX<double> v0_double = v0.template cast<double>();
  //const MatrixX<double> M0_double = M0.template cast<double>();
  //(void) M0;

  // Vector of unknowns, at k-th iteration.
  // X = [v; cn]
  VectorX<double> Xk = VectorX<double>::Zero(nv + num_contacts);
  // Aliases to different portions in Xk
  auto vk = Xk.segment(0, nv);
  auto cnk = Xk.segment(nv + 1, num_contacts);
  (void)cnk;
  // Reuse context_star for the NR iteration.
  //Context<double>& context_k = *context_star;
  //(void) context_k;

  // Initial guess for NR iteration.
  Xk.segment(0, nv) = v0;

  const int max_iterations = 20;
  const double tolerance = 1.0e-6;
  //const bool update_geometry_every_iteration = false;

  VectorX<double> Rk(Xk.size());
  MatrixX<double> Jk(Xk.size(), Xk.size());
  VectorX<double> DeltaXk(Xk.size());
  for (int iter = 0; iter < max_iterations; ++iter) {
    // Compute NR residual.
    // TODO(amcastro-tri): consider updating Nk = Nstar. This will however be
    // a more expensive operation.

    //R = CalcFischerBurmeisterSolverResidual(
      //  v0, M0, tau0, Nstar, VectorX<double>(vk), VectorX<double>(cnk));

    // Compute Residual and Jacobian.
    CalcFischerBurmeisterSolverJacobian(v0, M0, tau0, Nstar, vk, cnk, &Rk, &Jk);

    PRINT_VAR(iter);
    PRINT_VARn(num_contacts);
    PRINT_VARn(M0);
    PRINT_VAR(tau0.transpose());
    PRINT_VARn(Nstar);
    PRINT_VAR(Rk.transpose());
    PRINT_VARn(Jk);

    // Compute the complete orthogonal factorization of J.
    Eigen::CompleteOrthogonalDecomposition<MatrixX<double>> Jk_QTZ(Jk);

    // Solve
    DeltaXk = -Jk_QTZ.solve(Rk);

    // Update solution:
    Xk = Xk + DeltaXk;

    double residual = DeltaXk.segment(0, nv).norm();


    PRINT_VAR(residual);
    PRINT_VAR(vk.transpose());
    PRINT_VAR(Xk.transpose());

    if (residual < tolerance) {
      break;
    }
  }

  // Output solution
  vn = Xk.segment(0, nv);

  //vn = v0 + dt * M0.ldlt().solve(-tau0);

  VectorX<double> qdotn(this->num_positions());
  model_->MapVelocityToQDot(context0, vn, &qdotn);

  // qn = q + dt*qdot.
  VectorX<double> xn(this->num_multibody_states());
  xn << q0 + dt * qdotn, vn;
  updates->get_mutable_vector(0).SetFromVector(xn);
}

template<typename T>
void MultibodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>& events,
    drake::systems::DiscreteValues<T>* updates) const {
  DRAKE_ABORT_MSG("T != double not supported.");
}

template <typename T>
template <typename  T1>
typename std::enable_if<
    std::is_same<T1, double>::value,
    std::vector<geometry::PenetrationAsPointPair<T>>>::type
MultibodyPlant<T>::ComputePenetrations(
    const Context<T>& context) const {
  std::vector<PenetrationAsPointPair<T>> contact_penetrations;
  if (get_num_collision_geometries() == 0) return contact_penetrations;

  const geometry::QueryObject<double>& query_object =
      this->EvalAbstractInput(context, geometry_query_port_)
          ->template GetValue<geometry::QueryObject<double>>();
  std::vector<PenetrationAsPointPair<double>> penetrations =
      query_object.ComputePointPairPenetration();

  // Count the number of contacts using filtering from visuals
  for (const auto& penetration : penetrations) {
    const GeometryId geometryA_id = penetration.id_A;
    const GeometryId geometryB_id = penetration.id_B;
    if (is_collision_geometry(geometryA_id) &&
        is_collision_geometry(geometryB_id))
      contact_penetrations.push_back(penetration);
  }
  return contact_penetrations;
}

template <typename T>
template <typename  T1>
typename std::enable_if<
    !std::is_same<T1, double>::value,
    std::vector<geometry::PenetrationAsPointPair<T>>>::type
MultibodyPlant<T>::ComputePenetrations(const Context<T>&) const {
  DRAKE_ABORT_MSG("Only <double> is supported.");
}

// This method is assuming that we are giving a compatible `context` with a
// `contact_penetrations`, where each contact pair, in theory,
// has point_pair.depth = 0. That is, each contact pair is "exactly" at contact.
// However in practice these are usually computed with some finite penetration.
template<typename T>
MatrixX<T> MultibodyPlant<T>::ComputeNormalVelocityJacobianMatrix(
    const Context<T>& context,
    std::vector<PenetrationAsPointPair<T>>& contact_penetrations) const {
  const int num_contacts = contact_penetrations.size();
  MatrixX<T> N(num_contacts, num_velocities());

  for (int icontact = 0; icontact < num_contacts; ++icontact) {
    const auto& point_pair = contact_penetrations[icontact];

    const GeometryId geometryA_id = point_pair.id_A;
    const GeometryId geometryB_id = point_pair.id_B;

    // TODO(amcastro-tri): Request GeometrySystem to do this filtering for us
    // when that capability lands.
    // TODO(amcastro-tri): consider allowing this id's to belong to a third
    // external system when they correspond to anchored geometry.
    if (!is_collision_geometry(geometryA_id) ||
        !is_collision_geometry(geometryB_id))
      continue;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    const Body<T>& bodyA = model().get_body(bodyA_index);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);
    const Body<T>& bodyB = model().get_body(bodyB_index);

    // Penetration depth, > 0 during point_pair.
    const T& x = point_pair.depth;
    DRAKE_ASSERT(x >= 0);
    const Vector3<T>& nhat_BA_W = point_pair.nhat_BA_W;
    const Vector3<T>& p_WCa = point_pair.p_WCa;
    const Vector3<T>& p_WCb = point_pair.p_WCb;

    // Approximate the position of the contact point as:
    // In theory p_WC = p_WCa = p_WCb.
    const Vector3<T> p_WC = 0.5 * (p_WCa + p_WCb);  // notice this is at t_star.
    // TODO(amcastro-tri): for each contact point, consider computing
    // dtc = phi / phidot and then estimate the contact point as:
    //  p_WCa = p_WCa_star + dtc * v0_WCa
    //  p_WCb = p_WCb_star + dtc * v0_WCb
    // In theory, these two estimations should be very close to the actual p_WC.
    // Then do:
    //  p_WC = 0.5 * (p_WCa + p_WCb);

    MatrixX<T> Jv_WAc;  // s.t.: v_WAc = Jv_WAc * v.
    model().CalcPointsGeometricJacobianExpressedInWorld(
        context, bodyA.body_frame(), p_WC, &Jv_WAc);

    MatrixX<T> Jv_WBc;  // s.t.: v_WBc = Jv_WBc * v.
    model().CalcPointsGeometricJacobianExpressedInWorld(
        context, bodyB.body_frame(), p_WC, &Jv_WBc);

    // Therefore v_AcBc_W = v_WBc - v_WAc.
    // if xdot = vn > 0 ==> they are getting closer.
    // vn = v_AcBc_W.dot(nhat_BA_W);
    // vn = (nhat^T * J) * v
    N.row(icontact) = nhat_BA_W.transpose() * (Jv_WBc - Jv_WAc);
  }

  return N;
}

template<typename T>
void MultibodyPlant<T>::set_penetration_allowance(
    double penetration_allowance) {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  // Default to Earth's gravity for this estimation.
  const double g = gravity_field_.has_value() ?
                   gravity_field_.value()->gravity_vector().norm() : 9.81;

  // TODO(amcastro-tri): Improve this heuristics in future PR's for when there
  // are several flying objects and fixed base robots (E.g.: manipulation
  // cases.)

  // The heuristic now is very simple. We should update it to:
  //  - Only scan free bodies for weight.
  //  - Consider an estimate of maximum velocities (context dependent).
  // Right now we are being very conservative and use the maximum mass in the
  // system.
  double mass = 0.0;
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = model().get_body(body_index);
    mass = std::max(mass, body.get_default_mass());
  }

  // For now, we use the model of a critically damped spring mass oscillator
  // to estimate these parameters: mẍ+cẋ+kx=mg
  // Notice however that normal forces are computed according to: fₙ=kx(1+dẋ)
  // which translate to a second order oscillator of the form:
  // mẍ+(kdx)ẋ+kx=mg
  // Therefore, for this more complex, non-linear, oscillator, we estimate the
  // damping constant d using a time scale related to the free oscillation
  // (omega below) and the requested penetration allowance as a length scale.

  // We first estimate the stiffness based on static equilibrium.
  const double stiffness = mass * g / penetration_allowance;
  // Frequency associated with the stiffness above.
  const double omega = sqrt(stiffness / mass);

  // Estimated contact time scale. The relative velocity of objects coming into
  // contact goes to zero in this time scale.
  const double time_scale = 1.0 / omega;

  // Damping ratio for a critically damped model. We could allow users to set
  // this. Right now, critically damp the normal direction.
  // This corresponds to a non-penetraion constraint in the limit for
  // contact_penetration_allowance_ goint to zero (no bounce off).
  const double damping_ratio = 1.0;
  // We form the damping (with units of 1/velocity) using dimensional analysis.
  // Thus we use 1/omega for the time scale and penetration_allowance for the
  // length scale. We then scale it by the damping ratio.
  const double damping = damping_ratio * time_scale / penetration_allowance;

  // Final parameters used in the penalty method:
  penalty_method_contact_parameters_.stiffness = stiffness;
  penalty_method_contact_parameters_.damping = damping;
  // The time scale can be requested to hint the integrator's time step.
  penalty_method_contact_parameters_.time_scale = time_scale;
}

template<>
void MultibodyPlant<double>::CalcAndAddContactForcesByPenaltyMethod(
    const systems::Context<double>& context,
    const PositionKinematicsCache<double>& pc,
    const VelocityKinematicsCache<double>& vc,
    std::vector<SpatialForce<double>>* F_BBo_W_array) const {
  if (get_num_collision_geometries() == 0) return;

  const geometry::QueryObject<double>& query_object =
      this->EvalAbstractInput(context, geometry_query_port_)
          ->template GetValue<geometry::QueryObject<double>>();

  std::vector<PenetrationAsPointPair<double>> penetrations =
      query_object.ComputePointPairPenetration();
  for (const auto& penetration : penetrations) {
    const GeometryId geometryA_id = penetration.id_A;
    const GeometryId geometryB_id = penetration.id_B;

    // TODO(amcastro-tri): Request GeometrySystem to do this filtering for us
    // when that capability lands.
    // TODO(amcastro-tri): consider allowing this id's to belong to a third
    // external system when they correspond to anchored geometry.
    if (!is_collision_geometry(geometryA_id) ||
        !is_collision_geometry(geometryB_id))
      continue;

    BodyIndex bodyA_index = geometry_id_to_body_index_.at(geometryA_id);
    BodyIndex bodyB_index = geometry_id_to_body_index_.at(geometryB_id);

    BodyNodeIndex bodyA_node_index =
        model().get_body(bodyA_index).node_index();
    BodyNodeIndex bodyB_node_index =
        model().get_body(bodyB_index).node_index();

    // Penetration depth, > 0 during penetration.
    const double& x = penetration.depth;
    DRAKE_ASSERT(x >= 0);
    const Vector3<double>& nhat_BA_W = penetration.nhat_BA_W;
    const Vector3<double>& p_WCa = penetration.p_WCa;
    const Vector3<double>& p_WCb = penetration.p_WCb;

    // Contact point C.
    const Vector3<double> p_WC = 0.5 * (p_WCa + p_WCb);

    // Contact point position on body A.
    const Vector3<double>& p_WAo =
        pc.get_X_WB(bodyA_node_index).translation();
    const Vector3<double>& p_CoAo_W = p_WAo - p_WC;

    // Contact point position on body B.
    const Vector3<double>& p_WBo =
        pc.get_X_WB(bodyB_node_index).translation();
    const Vector3<double>& p_CoBo_W = p_WBo - p_WC;

    // Separation velocity, > 0  if objects separate.
    const Vector3<double> v_WAc =
        vc.get_V_WB(bodyA_node_index).Shift(-p_CoAo_W).translational();
    const Vector3<double> v_WBc =
        vc.get_V_WB(bodyB_node_index).Shift(-p_CoBo_W).translational();
    const Vector3<double> v_AcBc_W = v_WBc - v_WAc;

    // if xdot = vn > 0 ==> they are getting closer.
    const double vn = v_AcBc_W.dot(nhat_BA_W);

    // Magnitude of the normal force on body A at contact point C.
    const double k = penalty_method_contact_parameters_.stiffness;
    const double d = penalty_method_contact_parameters_.damping;
    const double fn_AC = k * x * (1.0 + d * vn);

    if (fn_AC > 0) {
      // Normal force on body A, at C, expressed in W.
      const Vector3<double> fn_AC_W = fn_AC * nhat_BA_W;

      // Since the normal force is positive (non-zero), compute the friction
      // force. First obtain the friction coefficients:
      const int collision_indexA =
          geometry_id_to_collision_index_.at(geometryA_id);;
      const int collision_indexB =
          geometry_id_to_collision_index_.at(geometryB_id);;
      const CoulombFriction<double>& geometryA_friction =
          default_coulomb_friction_[collision_indexA];
      const CoulombFriction<double>& geometryB_friction =
          default_coulomb_friction_[collision_indexB];
      const CoulombFriction<double> combined_friction_coefficients =
          CalcContactFrictionFromSurfaceProperties(
              geometryA_friction, geometryB_friction);
      // Compute tangential velocity, that is, v_AcBc projected onto the tangent
      // plane with normal nhat_BA:
      const Vector3<double> vt_AcBc_W = v_AcBc_W - vn * nhat_BA_W;
      // Tangential speed (squared):
      const double vt_squared = vt_AcBc_W.squaredNorm();

      // Consider a value indistinguishable from zero if it is smaller
      // then 1e-14 and test against that value squared.
      const double kNonZeroSqd = 1e-14 * 1e-14;
      // Tangential friction force on A at C, expressed in W.
      Vector3<double> ft_AC_W = Vector3<double>::Zero();
      if (vt_squared > kNonZeroSqd) {
        const double vt = sqrt(vt_squared);
        // Stribeck friction coefficient.
        const double mu_stribeck = stribeck_model_.ComputeFrictionCoefficient(
            vt, combined_friction_coefficients);
        // Tangential direction.
        const Vector3<double> that_W = vt_AcBc_W / vt;

        // Magnitude of the friction force on A at C.
        const double ft_AC = mu_stribeck * fn_AC;
        ft_AC_W = ft_AC * that_W;
      }

      // Spatial force on body A at C, expressed in the world frame W.
      const SpatialForce<double> F_AC_W(Vector3<double>::Zero(),
                                        fn_AC_W + ft_AC_W);

      if (F_BBo_W_array != nullptr) {
        if (bodyA_index != world_index()) {
          // Spatial force on body A at Ao, expressed in W.
          const SpatialForce<double> F_AAo_W = F_AC_W.Shift(p_CoAo_W);
          F_BBo_W_array->at(bodyA_index) += F_AAo_W;
        }

        if (bodyB_index != world_index()) {
          // Spatial force on body B at Bo, expressed in W.
          const SpatialForce<double> F_BBo_W = -F_AC_W.Shift(p_CoBo_W);
          F_BBo_W_array->at(bodyB_index) += F_BBo_W;
        }
      }
    }
  }
}

template<typename T>
void MultibodyPlant<T>::CalcAndAddContactForcesByPenaltyMethod(
    const systems::Context<T>&,
    const PositionKinematicsCache<T>&, const VelocityKinematicsCache<T>&,
    std::vector<SpatialForce<T>>*) const {
  DRAKE_ABORT_MSG("Only <double> is supported.");
}

template<typename T>
void MultibodyPlant<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    systems::VectorBase<T>* generalized_velocity) const {
  const int nq = model_->num_positions();
  const int nv = model_->num_velocities();

  DRAKE_ASSERT(qdot.size() == nq);
  DRAKE_DEMAND(generalized_velocity != nullptr);
  DRAKE_DEMAND(generalized_velocity->size() == nv);

  VectorX<T> v(nv);
  model_->MapQDotToVelocity(context, qdot, &v);
  generalized_velocity->SetFromVector(v);
}

template<typename T>
void MultibodyPlant<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    systems::VectorBase<T>* positions_derivative) const {
  const int nq = model_->num_positions();
  const int nv = model_->num_velocities();

  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_DEMAND(positions_derivative != nullptr);
  DRAKE_DEMAND(positions_derivative->size() == nq);

  VectorX<T> qdot(nq);
  model_->MapVelocityToQDot(context, generalized_velocity, &qdot);
  positions_derivative->SetFromVector(qdot);
}

template<typename T>
void MultibodyPlant<T>::DeclareStateAndPorts() {
  // The model must be finalized.
  DRAKE_DEMAND(this->is_finalized());

  if (is_state_discrete()) {
    this->DeclarePeriodicDiscreteUpdate(time_step_);
    this->DeclareDiscreteState(num_multibody_states());
  } else {
    this->DeclareContinuousState(
        BasicVector<T>(model_->num_states()),
        model_->num_positions(),
        model_->num_velocities(), 0 /* num_z */);
  }

  if (num_actuators() > 0) {
    actuation_port_ =
        this->DeclareVectorInputPort(
            systems::BasicVector<T>(num_actuated_dofs())).get_index();
  }

  continuous_state_output_port_ =
      this->DeclareVectorOutputPort(
          BasicVector<T>(num_multibody_states()),
          &MultibodyPlant::CopyContinuousStateOut).get_index();
}

template <typename T>
void MultibodyPlant<T>::CopyContinuousStateOut(
    const Context<T>& context, BasicVector<T>* state_vector) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  state_vector->SetFrom(context.get_continuous_state_vector());
}

template <typename T>
const systems::InputPortDescriptor<T>&
MultibodyPlant<T>::get_actuation_input_port() const {
  DRAKE_THROW_UNLESS(is_finalized());
  DRAKE_THROW_UNLESS(num_actuators() > 0);
  return systems::System<T>::get_input_port(actuation_port_);
}

template <typename T>
const systems::OutputPort<T>&
MultibodyPlant<T>::get_continuous_state_output_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  return this->get_output_port(continuous_state_output_port_);
}

template<typename T>
void MultibodyPlant<T>::DeclareGeometrySystemPorts() {
  geometry_query_port_ = this->DeclareAbstractInputPort().get_index();
  // This presupposes that the source id has been assigned and _all_ frames have
  // been registered.
  std::vector<FrameId> ids;
  for (auto it : body_index_to_frame_id_) {
    ids.push_back(it.second);
  }
  geometry_pose_port_ =
      this->DeclareAbstractOutputPort(
          FramePoseVector<T>(*source_id_, ids),
          &MultibodyPlant::CalcFramePoseOutput).get_index();
}

template <typename T>
void MultibodyPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_ASSERT(source_id_ != nullopt);
  DRAKE_ASSERT(
      poses->size() == static_cast<int>(body_index_to_frame_id_.size()));
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  // TODO(amcastro-tri): Make use of Body::EvalPoseInWorld(context) once caching
  // lands.
  poses->clear();
  for (const auto it : body_index_to_frame_id_) {
    const BodyIndex body_index = it.first;
    const Body<T>& body = model_->get_body(body_index);

    // NOTE: The GeometryFrames for each body were registered in the world
    // frame, so we report poses in the world frame.
    poses->set_value(body_index_to_frame_id_.at(body_index),
                     pc.get_X_WB(body.node_index()));
  }
}

template <typename T>
const OutputPort<T>& MultibodyPlant<T>::get_geometry_poses_output_port()
const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_DEMAND(geometry_source_is_registered());
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
const systems::InputPortDescriptor<T>&
MultibodyPlant<T>::get_geometry_query_input_port() const {
  DRAKE_MBP_THROW_IF_NOT_FINALIZED();
  DRAKE_DEMAND(geometry_source_is_registered());
  return systems::System<T>::get_input_port(geometry_query_port_);
}

template<typename T>
void MultibodyPlant<T>::DeclareCacheEntries() {
  // TODO(amcastro-tri): User proper System::Declare() infrastructure to
  // declare cache entries when that lands.
  pc_ = std::make_unique<PositionKinematicsCache<T>>(model_->get_topology());
  vc_ = std::make_unique<VelocityKinematicsCache<T>>(model_->get_topology());
}

template<typename T>
const PositionKinematicsCache<T>& MultibodyPlant<T>::EvalPositionKinematics(
    const systems::Context<T>& context) const {
  // TODO(amcastro-tri): Replace Calc() for an actual Eval() when caching lands.
  model_->CalcPositionKinematicsCache(context, pc_.get());
  return *pc_;
}

template<typename T>
const VelocityKinematicsCache<T>& MultibodyPlant<T>::EvalVelocityKinematics(
    const systems::Context<T>& context) const {
  // TODO(amcastro-tri): Replace Calc() for an actual Eval() when caching lands.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  model_->CalcVelocityKinematicsCache(context, pc, vc_.get());
  return *vc_;
}

template <typename T>
void MultibodyPlant<T>::ThrowIfFinalized(const char* source_method) const {
  if (is_finalized()) {
    throw std::logic_error(
        "Post-finalize calls to '" + std::string(source_method) + "()' are "
        "not allowed; calls to this method must happen before Finalize().");
  }
}

template <typename T>
void MultibodyPlant<T>::ThrowIfNotFinalized(const char* source_method) const {
  if (!is_finalized()) {
    throw std::logic_error(
        "Pre-finalize calls to '" + std::string(source_method) + "()' are "
        "not allowed; you must call Finalize() first.");
  }
}

template <typename T>
T MultibodyPlant<T>::StribeckModel::ComputeFrictionCoefficient(
    const T& speed_BcAc,
    const CoulombFriction<T>& friction) const {
  DRAKE_ASSERT(speed_BcAc >= 0);
  const T& mu_d = friction.dynamic_friction();
  const T& mu_s = friction.static_friction();
  const T v = speed_BcAc * inv_v_stiction_tolerance_;
  if (v >= 3) {
    return mu_d;
  } else if (v >= 1) {
    return mu_s - (mu_s - mu_d) * step5((v - 1) / 2);
  } else {
    return mu_s * step5(v);
  }
}

template <typename T>
T MultibodyPlant<T>::StribeckModel::step5(const T& x) {
  DRAKE_ASSERT(0 <= x && x <= 1);
  const T x3 = x * x * x;
  return x3 * (10 + x * (6 * x - 15));  // 10x³ - 15x⁴ + 6x⁵
}

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::multibody_plant::MultibodyPlant)
