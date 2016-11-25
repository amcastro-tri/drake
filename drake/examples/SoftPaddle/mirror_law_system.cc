#include "drake/examples/SoftPaddle/mirror_law_system.h"

#include "drake/examples/SoftPaddle/soft_paddle_state_vector.h"

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

template <typename T>
PaddleMirrorLawSystem<T>::PaddleMirrorLawSystem(const T& phi0, const T& amplitude) :
    phi0_(phi0), amplitude_(amplitude) {
  // Input for the SoftPaddlePlant state.
  this->DeclareInputPort(kVectorValued,
                         SoftPaddleStateVectorIndices::kNumCoordinates,
                         kContinuousSampling);
  // Output for the commanded paddle angle.
  this->DeclareOutputPort(kVectorValued, 1, kContinuousSampling);
}

template <typename T>
const SystemPortDescriptor<T>&
PaddleMirrorLawSystem<T>::get_paddle_angle_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void PaddleMirrorLawSystem<T>::EvalOutput(const Context<T>& context,
                                         SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  //
  //const auto paddle_state = this->EvalVectorInput(context, 0);
  auto paddle_state =
      dynamic_cast<const SoftPaddleStateVector<T>*>(this->EvalVectorInput(context, 0));

  System<T>::GetMutableOutputVector(output, 0)(0) =
      phi0_ + amplitude_ * paddle_state->zdot();

  //auto input_vector = this->EvalEigenVectorInput(context, 0);
 // System<T>::GetMutableOutputVector(output, 0) =
   //   k_.array() * input_vector.array();
}

// Explicitly instantiates on the most common scalar types.
template class PaddleMirrorLawSystem<double>;
template class PaddleMirrorLawSystem<AutoDiffXd>;

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
