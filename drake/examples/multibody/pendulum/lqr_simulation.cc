#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/multibody/pendulum/pendulum_plant.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {
namespace examples {
namespace multibody {
namespace pendulum {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_string(integration_scheme, "runge_kutta3",
              "Integration scheme to be used. Available options are:"
              "'runge_kutta3','implicit_euler','semi_explicit_euler'");

using geometry::GeometrySystem;
using geometry::SourceId;
using lcm::DrakeLcm;
using systems::AffineSystem;
using systems::BasicVector;
using systems::Context;
using systems::ImplicitEulerIntegrator;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
using systems::rendering::PoseBundleToDrawMessage;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;

std::unique_ptr<AffineSystem<double>> MakeLinearQuadraticRegulator(
    double mass, double length, double gravity) {
  PendulumPlant<double> pendulum(mass, length, gravity);

  // Prepare to linearize around the vertical equilibrium point (with tau=0)
  auto pendulum_context = pendulum.CreateDefaultContext();
  pendulum.SetAngle(pendulum_context.get(), M_PI);
  auto input = std::make_unique<PendulumInput<double>>();
  input->set_tau(0.0);
  pendulum_context->FixInputPort(0, std::move(input));

  // Set up cost function for LQR: integral of 10*theta^2 + thetadot^2 + tau^2.
  // The factor of 10 is heuristic, but roughly accounts for the unit conversion
  // between angles and angular velocity (using the time constant, \sqrt{g/l},
  // squared).
  Eigen::MatrixXd Q(2, 2);
  Q << 10, 0, 0, 1;
  Eigen::MatrixXd R(1, 1);
  R << 1;

  return systems::controllers::LinearQuadraticRegulator(
      pendulum, *pendulum_context, Q, R);
}

int do_main() {
  // Define plant's parameters:
  const double mass = 0.5;      // [Kgr], about a pound.
  const double length = 0.7;    // [m]
  const double gravity = 9.81;  // [m/s²]

  // Define simulation parameters:
  // Compute a reference time scale to set reasonable values for time step and
  // simulation time.
  const double reference_time_scale =
      PendulumPlant<double>::SimplePendulumPeriod(length, gravity);

  // Define a reasonable maximum time step based off the expected dynamics's
  // time scales.
  const double max_time_step = reference_time_scale / 100;

  // Simulate about five periods of oscillation.
  const double simulation_time = 5.0 * reference_time_scale;

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = 0.001;

  // Set up Diagram for simulation:
  systems::DiagramBuilder<double> builder;

  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");

  // Add a pendulum plant for simulation that registers geometry in
  // geometry_system for visualization.
  auto pendulum = builder.AddSystem<PendulumPlant>(
      mass, length, gravity, geometry_system);
  pendulum->set_name("Pendulum");

  // Make and add an LQR controller system.
  auto controller =
      builder.AddSystem(MakeLinearQuadraticRegulator(mass, length, gravity));
  controller->set_name("controller");
  builder.Connect(
      pendulum->get_state_output_port(), controller->get_input_port());
  builder.Connect(controller->get_output_port(), pendulum->get_input_port());

  // Boilerplate used to connect the plant to a GeometrySystem for
  // visualization.
  DrakeLcm lcm;
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_publish_period(1 / 60.0);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!pendulum->get_source_id());

  builder.Connect(
      pendulum->get_geometry_id_output_port(),
      geometry_system->get_source_frame_id_port(
          pendulum->get_source_id().value()));
  builder.Connect(
      pendulum->get_geometry_pose_output_port(),
      geometry_system->get_source_pose_port(pendulum->get_source_id().value()));

  builder.Connect(geometry_system->get_pose_bundle_output_port(),
                  converter->get_input_port(0));
  builder.Connect(*converter, *publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(*geometry_system);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& pendulum_context =
      diagram->GetMutableSubsystemContext(*pendulum,
                                          &simulator.get_mutable_context());

  // Set the state to be slightly off the fixed point:
  pendulum->SetAngle(&pendulum_context, M_PI + 0.5);
  pendulum->set_angular_velocity(&pendulum_context, 0.2);

  systems::IntegratorBase<double>* integrator;
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<RungeKutta3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  } else {
    throw std::logic_error(
        "Integration scheme not supported for this example.");
  }
  integrator->set_maximum_step_size(max_time_step);

  // Error control is only supported for variable time step integrators.
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(target_accuracy);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(simulation_time);

  // Some sanity checks:
  if (FLAGS_integration_scheme == "semi_explicit_euler") {
    DRAKE_DEMAND(integrator->get_fixed_step_mode() == true);
  }

  // Checks for variable time step integrators.
  if (!integrator->get_fixed_step_mode()) {
    DRAKE_DEMAND(integrator->get_largest_step_size_taken() <= max_time_step);
    DRAKE_DEMAND(integrator->get_smallest_adapted_step_size_taken() <=
        integrator->get_largest_step_size_taken());
    DRAKE_DEMAND(
        integrator->get_num_steps_taken() >= simulation_time / max_time_step);
  }

  // Checks for fixed time step integrators.
  if (integrator->get_fixed_step_mode()) {
    DRAKE_DEMAND(integrator->get_num_derivative_evaluations() ==
        integrator->get_num_steps_taken());
    DRAKE_DEMAND(
        integrator->get_num_step_shrinkages_from_error_control() == 0);
  }

  // We made a good guess for max_time_step and therefore we expect no
  // failures when taking a time step.
  DRAKE_DEMAND(integrator->get_num_substep_failures() == 0);
  DRAKE_DEMAND(
      integrator->get_num_step_shrinkages_from_substep_failures() == 0);

  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::pendulum::do_main();
}
