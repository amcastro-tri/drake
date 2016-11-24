#include "drake/examples/SoftPaddle/soft_paddle_plant.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace examples {
namespace soft_paddle {

namespace {
// q = [x, z, phi], xc = [q, qdot]
constexpr int kStateSize = 6;

// xc = [phi, phidot]
constexpr int kVisualizerStateSize = 2;
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
  output->GetMutableVectorData(1)->SetAtIndex(0, M_PI*context.get_time()/5.);
  output->GetMutableVectorData(1)->SetAtIndex(1, 0.);
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

  // Disk coordinates in paddle's frame (assuming now phi = 0)
  T x_p = x;
  T z_p = z;

  T delta = Rd_ - z_p;  // Penetration distance into undeformed paddle.
  // Compute elastic interaction force.
  T Fx = 0.;  // Horizontal force in the world's frame.
  T Fz = 0.;  // Vertical force in the world's frame.
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

    // Force in worlds's frame (assuming now phi = 0)
    Fx = Fx_p;
    Fz = Fz_p;
  }

  // Disk's acceleration.
  derivative_vector->set_xdot(Fx);
  derivative_vector->set_zdot((Fz - md_ * g_));

  // For now ignore paddle's dynamics.
  T tau = get_tau(context);
  (void) tau;
  derivative_vector->set_phidot(0.0);

  // REMOVE THIS!!!!
  derivative_vector->set_value(Vector6<T>::Constant(0.0));
}

// SoftPaddlePlant has no constructor arguments, so there's no work to do here.
template <typename T>
SoftPaddlePlant<AutoDiffXd>* SoftPaddlePlant<T>::DoToAutoDiffXd() const {
  return new SoftPaddlePlant<AutoDiffXd>();
}

template class SoftPaddlePlant<double>;
template class SoftPaddlePlant<AutoDiffXd>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
