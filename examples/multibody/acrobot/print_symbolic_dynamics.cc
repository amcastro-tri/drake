#include <iostream>

#include "drake/common/symbolic.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"

// A simple example of extracting the symbolic dynamics of the pendulum system,
// and printing them to std::out.

namespace drake {

using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::multibody_plant::MultibodyPlant;
using multibody::RevoluteJoint;

namespace examples {
namespace multibody {
namespace acrobot {
namespace {

int DoMain() {
  // The model's parameters:
  AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> system_on_double =
      MakeAcrobotPlant(parameters, true);
  std::unique_ptr<MultibodyPlant<symbolic::Expression>> system_on_symbolic =
      systems::System<double>::ToSymbolic(*system_on_double);

  DRAKE_DEMAND(system_on_double->num_positions() == 2);
  DRAKE_DEMAND(system_on_double->num_velocities() == 2);

  auto context = system_on_symbolic->CreateDefaultContext();
  context->FixInputPort(
      system_on_symbolic->get_actuation_input_port().get_index(),
      Vector1<symbolic::Expression>::Constant(symbolic::Variable("tau")));
  context->get_mutable_continuous_state_vector().SetAtIndex(
      0, symbolic::Variable("theta1"));
  context->get_mutable_continuous_state_vector().SetAtIndex(
      1, symbolic::Variable("theta2"));
  context->get_mutable_continuous_state_vector().SetAtIndex(
      2, symbolic::Variable("theta1dot"));
  context->get_mutable_continuous_state_vector().SetAtIndex(
      3, symbolic::Variable("theta2dot"));

  auto derivatives = system_on_symbolic->AllocateTimeDerivatives();
  system_on_symbolic->CalcTimeDerivatives(*context, derivatives.get());

  std::cout << derivatives->CopyToVector() << "\n";

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::multibody::acrobot::DoMain();
}
