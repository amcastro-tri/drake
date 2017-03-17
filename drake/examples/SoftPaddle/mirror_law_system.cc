#include "drake/examples/SoftPaddle/mirror_law_system.h"

#include "drake/examples/SoftPaddle/soft_paddle_plant.h"
#include "drake/examples/SoftPaddle/soft_paddle_state_to_bot_visualizer.h"
#include "drake/examples/SoftPaddle/soft_paddle_state_vector.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/primitives/first_order_low_pass_filter.h"

namespace drake {
namespace examples {
namespace soft_paddle {

using systems::Context;
using systems::kVectorValued;
using systems::InputPortDescriptor;
using systems::OutputPortDescriptor;
using systems::System;
using systems::SystemOutput;
using systems::FirstOrderLowPassFilter;

template <typename T>
PaddleMirrorLawSystem<T>::PaddleMirrorLawSystem(const T& phi0, const T& amplitude) :
    phi0_(phi0), amplitude_(amplitude) {
  // Input for the SoftPaddlePlant state.
  paddle_state_input_ =
      this->DeclareInputPort(
          kVectorValued,
          SoftPaddleStateVectorIndices::kNumCoordinates).get_index();

  // Input for law parameters.
  parameters_input_ =
      this->DeclareInputPort(kVectorValued, 2).get_index();

  // Output for the commanded paddle angle.
  this->DeclareOutputPort(kVectorValued, 1);
}

template <typename T>
const InputPortDescriptor<T>&
PaddleMirrorLawSystem<T>::get_parameters_input() const {
  return System<T>::get_input_port(parameters_input_);
}

template <typename T>
const OutputPortDescriptor<T>&
PaddleMirrorLawSystem<T>::get_paddle_angle_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void PaddleMirrorLawSystem<T>::DoCalcOutput(const Context<T>& context,
                                            SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  auto parameters = this->EvalEigenVectorInput(context, parameters_input_);
  const T& phi0 = parameters(0);
  const T& amplitude = parameters(1);
  (void) phi0;
  (void) amplitude;
  //
  //const auto paddle_state = this->EvalVectorInput(context, 0);
  auto paddle_state =
      dynamic_cast<const SoftPaddleStateVector<T>*>(
          this->EvalVectorInput(context, paddle_state_input_));

  System<T>::GetMutableOutputVector(output, 0)(0) =
      phi0_ + amplitude_ * paddle_state->zdot();

  //auto input_vector = this->EvalEigenVectorInput(context, 0);
 // System<T>::GetMutableOutputVector(output, 0) =
   //   k_.array() * input_vector.array();
}

template <typename T>
ApexMonitor<T>::ApexMonitor() {
  // Input for the SoftPaddlePlant state.
  paddle_state_input_port_ =
      this->DeclareInputPort(
          kVectorValued,
          SoftPaddleStateVectorIndices::kNumCoordinates).get_index();
  // Output for the commanded mirror law parameters.
  mirror_law_parameters_output_port_ =
      this->DeclareOutputPort(kVectorValued, 2).get_index();
}

template <typename T>
const InputPortDescriptor<T>&
ApexMonitor<T>::get_paddle_state_input() const {
  return System<T>::get_input_port(paddle_state_input_port_);
}

template <typename T>
const OutputPortDescriptor<T>&
ApexMonitor<T>::get_mirror_law_parameters_output() const {
  return System<T>::get_output_port(mirror_law_parameters_output_port_);
}

template <typename T>
void ApexMonitor<T>::DoCalcOutput(const Context<T>& context,
                                  SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  auto paddle_state =
      dynamic_cast<const SoftPaddleStateVector<T>*>(
          this->EvalVectorInput(context, paddle_state_input_port_));

  const T& w = paddle_state->zdot();

  if (w < 0 && w0_ >0) {
    std::cout << "Got to the Apex!" << std::endl;
  }

  w0_ = w;

  //System<T>::GetMutableOutputVector(output, 0)(0) =
  //    phi0_ + amplitude_ * paddle_state->zdot();
  //auto input_vector = this->EvalEigenVectorInput(context, 0);
  // System<T>::GetMutableOutputVector(output, 0) =
  //   k_.array() * input_vector.array();
}

template <typename T>
SoftPaddleWithMirrorControl<T>::SoftPaddleWithMirrorControl(
    const T& phi0, const T& amplitude, bool filter_commanded_angle) {
  systems::DiagramBuilder<T> builder;

  auto mirror_system =
      builder.template AddSystem<PaddleMirrorLawSystem>(phi0, amplitude);
  paddle_ = builder.template AddSystem<SoftPaddlePlant>();

  // Feedback loop.
  builder.Connect(paddle_->get_output_port(), mirror_system->get_input_port(0));
  if (filter_commanded_angle) {
    const double filter_time_constant = 0.15;  // In seconds.
    auto filter =
        builder.template AddSystem<FirstOrderLowPassFilter>(filter_time_constant);
    builder.Connect(
        mirror_system->get_paddle_angle_port(), filter->get_input_port());
    builder.Connect(
        filter->get_output_port(), paddle_->get_tau_port());
    command_angle_output_port_ =
        builder.ExportOutput(filter->get_output_port());
  } else {
    builder.Connect(
        mirror_system->get_paddle_angle_port(), paddle_->get_tau_port());
    builder.ExportOutput(mirror_system->get_paddle_angle_port());
  }

  state_output_port_ = builder.ExportOutput(paddle_->get_output_port());
  viz_elements_output_port_ =
      builder.ExportOutput(paddle_->get_elements_port());

  auto apex_monitor =
      builder.template AddSystem<ApexMonitor>();
  builder.Connect(paddle_->get_output_port(),
                  apex_monitor->get_paddle_state_input());

  builder.Connect(apex_monitor->get_mirror_law_parameters_output(),
                  mirror_system->get_parameters_input());


  builder.BuildInto(this);
}

template <typename T>
const OutputPortDescriptor<T>&
SoftPaddleWithMirrorControl<T>::get_paddle_state_port() const {
  return System<T>::get_output_port(state_output_port_);
}

template <typename T>
const OutputPortDescriptor<T>&
SoftPaddleWithMirrorControl<T>::get_paddle_angle_port() const {
  return System<T>::get_output_port(command_angle_output_port_);
}

template <typename T>
const OutputPortDescriptor<T>&
SoftPaddleWithMirrorControl<T>::get_elements_port() const {
  return System<T>::get_output_port(viz_elements_output_port_);
}

template <typename T>
void SoftPaddleWithMirrorControl<T>::set_initial_conditions(
    Context<T>* context, const T& x0, const T& z0) const {
  Context<T>* paddle_context = this->GetMutableSubsystemContext(context,
                                                               paddle_);
  auto state =
      dynamic_cast<SoftPaddleStateVector<T>*>(
          paddle_context->get_mutable_continuous_state_vector());
  state->SetFromVector(VectorX<T>::Zero(state->size()));
  state->set_x(x0);
  state->set_z(z0);
}

template <typename T>
SoftPaddleStateVector<T>*
    SoftPaddleWithMirrorControl<T>::GetMutablePlantStateVector(
        Context<T>* context) const {
  Context<T>* paddle_context = this->GetMutableSubsystemContext(context,
                                                                paddle_);
  return dynamic_cast<SoftPaddleStateVector<T>*>(
      paddle_context->get_mutable_continuous_state_vector());
}

// Explicitly instantiates on the most common scalar types.
template class PaddleMirrorLawSystem<double>;
// Eigen's tan fails at runtime if using AutoDiffXd.
// As a quick fix I am using a fixed size AutoDiffScalar.
template class PaddleMirrorLawSystem<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class PaddleMirrorLawSystem<Eigen::AutoDiffScalar<Eigen::Vector4d>>;
template class PaddleMirrorLawSystem<AutoDiffXd>;

template class SoftPaddleWithMirrorControl<double>;
// Eigen's tan fails at runtime if using AutoDiffXd.
// As a quick fix I am using a fixed size AutoDiffScalar.
template class SoftPaddleWithMirrorControl<
    Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class SoftPaddleWithMirrorControl<
    Eigen::AutoDiffScalar<Eigen::Vector4d>>;
template class SoftPaddleWithMirrorControl<AutoDiffXd>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
