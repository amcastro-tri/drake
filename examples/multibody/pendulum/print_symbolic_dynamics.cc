#include <iostream>

#include "drake/common/symbolic.h"
#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"

// A simple example of extracting the symbolic dynamics of the pendulum system,
// and printing them to std::out.

namespace drake {

using multibody::benchmarks::pendulum::MakePendulumPlant;
using multibody::benchmarks::pendulum::PendulumParameters;
using multibody::multibody_plant::MultibodyPlant;
using multibody::RevoluteJoint;

namespace examples {
namespace multibody {
namespace pendulum {
namespace {

int DoMain() {
  // The model's parameters:
  PendulumParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> system_on_double =
      MakePendulumPlant(parameters);
  std::unique_ptr<MultibodyPlant<symbolic::Expression>> system_on_symbolic =
      systems::System<double>::ToSymbolic(*system_on_double);

  auto context = system_on_symbolic->CreateDefaultContext();
  context->FixInputPort(
      system_on_symbolic->get_actuation_input_port().get_index(),
      Vector1<symbolic::Expression>::Constant(symbolic::Variable("tau")));
  context->get_mutable_continuous_state_vector().SetAtIndex(
      0, symbolic::Variable("theta"));
  context->get_mutable_continuous_state_vector().SetAtIndex(
      1, symbolic::Variable("thetadot"));

  auto derivatives = system_on_symbolic->AllocateTimeDerivatives();
  system_on_symbolic->CalcTimeDerivatives(*context, derivatives.get());

  std::cout << derivatives->CopyToVector() << "\n";

  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::multibody::pendulum::DoMain();
}
