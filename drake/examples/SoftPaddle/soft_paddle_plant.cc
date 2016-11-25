#include "drake/examples/SoftPaddle/soft_paddle_plant.h"

#include <memory>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/joints/revolute_joint.h"

namespace drake {
namespace examples {
namespace soft_paddle {

using std::make_unique;
using std::unique_ptr;

using DrakeShapes::Box;
using DrakeShapes::Cylinder;
using DrakeShapes::VisualElement;

using Eigen::Isometry3d;
using Eigen::Vector3d;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace {
// q = [x, z, phi], xc = [q, qdot]
constexpr int kStateSize = 6;

// 1 revolute joint for the paddle = 2 states.
// 1 quaternion joint for the disk = 13 (= 7 + 6) states.
constexpr int kVisualizerStateSize = 15;
}

template <typename T>
SoftPaddlePlant<T>::SoftPaddlePlant() {
  // Input port for the actuating torque on the paddle.
  this->DeclareInputPort(
      systems::kVectorValued, 1, systems::kContinuousSampling);

  // Outputs the state.
  this->DeclareOutputPort(
      systems::kVectorValued, kStateSize, systems::kContinuousSampling);

  // Output for BotVisualizer. (output includes generalized velocities)
  this->DeclareOutputPort(
      systems::kVectorValued, kVisualizerStateSize, systems::kContinuousSampling);

  CreateRBTModel();
}

template <typename T>
SoftPaddlePlant<T>::~SoftPaddlePlant() {}

template <typename T>
const systems::SystemPortDescriptor<T>&
SoftPaddlePlant<T>::get_tau_port() const {
  return this->get_input_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
SoftPaddlePlant<T>::get_output_port() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
SoftPaddlePlant<T>::get_visualizer_output_port() const {
  return systems::System<T>::get_output_port(1);
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
SoftPaddlePlant<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  if(descriptor.get_index() == 0) {
    return std::make_unique<SoftPaddleStateVector<T>>();
  } else {
    return std::make_unique<systems::BasicVector<T>>(kVisualizerStateSize);
  }
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
SoftPaddlePlant<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<SoftPaddleStateVector<T>>(),
      3 /* num_q */, 3 /* num_v */, 0 /* num_z */);
  static_assert(kStateSize == 3 + 3, "State size has changed");
}

template <typename T>
void SoftPaddlePlant<T>::EvalOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  // Set output 0: state.
  get_mutable_output(output)->set_value(get_state(context).get_value());
  // Set output 1: BotVisualizer
  const SoftPaddleStateVector<T>& state = get_state(context);
  // Revolute joint.
  output->GetMutableVectorData(1)->SetAtIndex(0, state.phi());
  // Quaternion for the disk.
  output->GetMutableVectorData(1)->SetAtIndex(1, state.x()-x0);
  output->GetMutableVectorData(1)->SetAtIndex(2, 0.0);
  output->GetMutableVectorData(1)->SetAtIndex(3, state.z()-z0);
  output->GetMutableVectorData(1)->SetAtIndex(4, 1.0);
  output->GetMutableVectorData(1)->SetAtIndex(5, 0.0);
  output->GetMutableVectorData(1)->SetAtIndex(6, 0.0);
  output->GetMutableVectorData(1)->SetAtIndex(7, 0.0);
}

// Compute the actual physics.
template <typename T>
void SoftPaddlePlant<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  const SoftPaddleStateVector<T>& state = get_state(context);
  SoftPaddleStateVector<T>* derivative_vector = get_mutable_state(derivatives);

  derivative_vector->set_x(state.xdot());
  derivative_vector->set_z(state.zdot());
  derivative_vector->set_phi(state.phidot());

  // =========================================================
  // Only consider disk's dynamics. And assume phi = 0.
  // =========================================================

  // Disk coordinates in world's frame.
  const T x = state.x();
  const T z = state.z();

  const T xdot = state.xdot();
  const T zdot = state.zdot();

  // Disk coordinates in paddle's frame (assuming now phi = 0)
  T x_p = x;
  T z_p = z;

  T xdot_p = xdot; (void) xdot_p;
  T zdot_p = zdot;

  T delta = z_p - Rd_;  // Penetration distance into undeformed paddle.
  // Compute elastic interaction force.
  T Fx = 0.;  // Horizontal force in the world's frame.
  T Fz = 0.;  // Vertical force in the world's frame.

  //PRINT_VAR(x);
  //PRINT_VAR(z);
  //PRINT_VAR(delta);

  if(delta < 0) {

    // Angle between the rubber band and the horizontal on the left side.
    T theta1 = - delta / x_p;  // Always positive.

    // Angle between the rubber band and the horizontal on the right side.
    T theta2 = - delta / (ell_ - x_p);  // Always positive.

    // Angle between the two straight sections of the rubber band at each side
    // of the disk.
    T theta = theta1 + theta2;

    // Angle between the contact force and the vertical.
    // x > ell_ / 2 ==> beta > 0. Force points to the left.
    // x < ell_ / 2 ==> beta < 0. Force points to the right.
    T beta = (theta2 - theta1) / 2.0;

    // Forces acting on the disk in the frame of the paddle.
    T Fz_p = T0_ * theta;
    T Fx_p = - Fz_p * beta;

    // Damping force (find a better model).
    Fz_p += -damping_coefficient_ * zdot_p;

    // Force in worlds's frame (assuming now phi = 0)
    Fx = Fx_p;
    Fz = Fz_p;
  }

  //PRINT_VAR(Fx);
 // PRINT_VAR(Fz);

  // Disk's acceleration.
  derivative_vector->set_xdot(Fx);
  derivative_vector->set_zdot((Fz - md_ * g_));

  // For now ignore paddle's dynamics.
  T tau = get_tau(context);
  (void) tau;
  derivative_vector->set_phidot(0.0);

  // REMOVE THIS!!!!
  //derivative_vector->set_value(Vector6<T>::Constant(0.0));
}

// SoftPaddlePlant has no constructor arguments, so there's no work to do here.
template <typename T>
SoftPaddlePlant<AutoDiffXd>* SoftPaddlePlant<T>::DoToAutoDiffXd() const {
  return new SoftPaddlePlant<AutoDiffXd>();
}

template <typename T>
void SoftPaddlePlant<T>::CreateRBTModel() {
  rbt_model_ = std::make_unique<RigidBodyTree<double>>();

  Eigen::Vector4d red(0.9, 0.1, 0.0, 1.0);
  Eigen::Vector4d green(0.3, 0.6, 0.4, 1.0);
  Eigen::Vector4d grey(0.5, 0.5, 0.5, 1.0);

  // Adds a RigidBody to model the paddle (only the rigid part).
  {
    RigidBody<double> *body =
        rbt_model_->add_rigid_body(make_unique<RigidBody<double>>());
    body->set_name("body");
    // Sets body to have a non-zero spatial inertia. Otherwise the body gets
    // welded by a fixed joint to the world by RigidBodyTree::compile().
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    Isometry3d pose = Isometry3d::Identity();
    pose.translation() = Vector3d(0.35, 0.0, 0.0);
    DrakeShapes::VisualElement visual_element(
        Box(Vector3d(ell_, 0.1, 0.03)), pose, red);
    body->AddVisualElement(visual_element);

    // Shaft visual.
    pose = Isometry3d::Identity();
    pose.linear() =
        Eigen::AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();
    DrakeShapes::VisualElement shaft_visual(
        Cylinder(0.03, 0.15), pose, grey);
    body->AddVisualElement(shaft_visual);

    // Paddle angle.
    body->add_joint(&rbt_model_->world(),
                      make_unique<RevoluteJoint>(
                          "phi", Isometry3d::Identity(), -Vector3d::UnitY()));
  }

  // Adds a RigidBody to model the disk.
  {
    RigidBody<double> *body =
        rbt_model_->add_rigid_body(make_unique<RigidBody<double>>());
    body->set_name("disk");
    // Sets body to have a non-zero spatial inertia. Otherwise the body gets
    // welded by a fixed joint to the world by RigidBodyTree::compile().
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());

    Isometry3d pose = Isometry3d::Identity();
    pose.translation() = Vector3d(x0, 0.0, z0);
    pose.linear() =
        Eigen::AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();
    DrakeShapes::VisualElement visual_element(
        Cylinder(Rd_, 0.025), pose, green);
    body->AddVisualElement(visual_element);

    body->add_joint(&rbt_model_->world(),
                    make_unique<QuaternionFloatingJoint>(
                        "phi", Isometry3d::Identity()));
  }

  rbt_model_->compile();
}

template <typename T>
void SoftPaddlePlant<T>::set_initial_conditions(MyContext* context) const {
  auto state =
      dynamic_cast<SoftPaddleStateVector<T>*>(
          context->get_mutable_continuous_state_vector());
  state->SetFromVector(VectorX<T>::Zero(kStateSize));
  state->set_x(x0);
  state->set_z(z0);
}

template class SoftPaddlePlant<double>;
template class SoftPaddlePlant<AutoDiffXd>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
