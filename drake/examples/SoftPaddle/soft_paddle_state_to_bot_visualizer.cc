#include "drake/examples/SoftPaddle/soft_paddle_state_to_bot_visualizer.h"

#include "drake/examples/SoftPaddle/soft_paddle_state_vector.h"
#include "drake/multibody/rigid_body_tree.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace examples {
namespace soft_paddle {

using systems::Context;
using systems::kVectorValued;
using systems::kContinuousSampling;
using systems::SystemPortDescriptor;
using systems::System;
using systems::SystemOutput;

namespace {
// q = [x, z], xc = [q, qdot]
constexpr int kStateSize = SoftPaddleStateVectorIndices::kNumCoordinates;

// 1 revolute joint for the paddle = 2 states.
// 1 quaternion joint for the disk = 13 (= 7 + 6) states.
constexpr int kVisualizerStateSize = 15;
}

template <typename T>
SoftPaddleStateToBotVisualizer<T>::SoftPaddleStateToBotVisualizer(
    const SoftPaddlePlant<T>& plant) :
    rbt_model_(plant.get_rigid_body_tree_model()),
    x0_(plant.get_default_x0()),
    z0_(plant.get_default_z0()){
  // Input for the SoftPaddlePlant state.
  this->DeclareInputPort(kVectorValued,
                         kStateSize,
                         kContinuousSampling);
  // Input for the paddle angle.
  this->DeclareInputPort(kVectorValued,
                         1,
                         kContinuousSampling);
  // Output for the BotVisualizer.
  this->DeclareOutputPort(kVectorValued, kVisualizerStateSize, kContinuousSampling);
}

template <typename T>
const SystemPortDescriptor<T>&
SoftPaddleStateToBotVisualizer<T>::get_paddle_angle_port() const {
  return System<T>::get_input_port(1);
}

template <typename T>
const SystemPortDescriptor<T>&
SoftPaddleStateToBotVisualizer<T>::get_paddle_state_port() const {
  return System<T>::get_input_port(0);
}

template <typename T>
const SystemPortDescriptor<T>&
SoftPaddleStateToBotVisualizer<T>::get_bot_visualizer_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void SoftPaddleStateToBotVisualizer<T>::EvalOutput(const Context<T>& context,
                                         SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Input from the paddle plant.
  const SoftPaddleStateVector<T>* state =
      dynamic_cast<const SoftPaddleStateVector<T>*>(
          this->EvalVectorInput(context, 0));

  // Input angle.
  T phi = this->EvalVectorInput(context, 1)->GetAtIndex(0);

  // Revolute joint.
  output->GetMutableVectorData(0)->SetAtIndex(0, phi);
  // Quaternion for the disk.
  output->GetMutableVectorData(0)->SetAtIndex(1, state->x() - x0_);
  output->GetMutableVectorData(0)->SetAtIndex(2, 0.0);
  output->GetMutableVectorData(0)->SetAtIndex(3, state->z() - z0_);
  output->GetMutableVectorData(0)->SetAtIndex(4, 1.0);
  output->GetMutableVectorData(0)->SetAtIndex(5, 0.0);
  output->GetMutableVectorData(0)->SetAtIndex(6, 0.0);
  output->GetMutableVectorData(0)->SetAtIndex(7, 0.0);
}

// Explicitly instantiates on the most common scalar types.
template class SoftPaddleStateToBotVisualizer<double>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
