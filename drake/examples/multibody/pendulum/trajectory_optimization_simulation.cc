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
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

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
using solvers::SolutionResult;
using systems::BasicVector;
using systems::Context;
using systems::ImplicitEulerIntegrator;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
using systems::rendering::PoseBundleToDrawMessage;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;

template <typename T>
class PidControlledSystem : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PidControlledSystem)

  /// @p plant full state is used for feedback control, and all the dimensions
  /// have homogeneous gains specified by @p Kp, @p Kd and @p Ki.
  ///
  /// @param[in] plant The system to be controlled. This must not be `nullptr`.
  /// @param[in] Kp the proportional constant.
  /// @param[in] Ki the integral constant.
  /// @param[in] Kd the derivative constant.
  PidControlledSystem(
      std::unique_ptr<PendulumPlant<T>> plant,
      int plant_input_port_index,
      systems::OutputPortIndex plant_state_port_index,
      double Kp, double Ki, double Kd) {
    systems::DiagramBuilder<T> builder;
    plant_ = builder.template AddSystem(std::move(plant));
    DRAKE_ASSERT(plant_->get_num_input_ports() >= 1);
    DRAKE_ASSERT(plant_->get_num_output_ports() >= 1);

    const auto& plant_input_port =plant_->get_input_port();
        //plant_->get_input_port(plant_input_port_index);
    const int input_size = plant_input_port.size();
    const Eigen::VectorXd Kp_v = Eigen::VectorXd::Ones(input_size) * Kp;
    const Eigen::VectorXd Ki_v = Eigen::VectorXd::Ones(input_size) * Ki;
    const Eigen::VectorXd Kd_v = Eigen::VectorXd::Ones(input_size) * Kd;

    auto controller =
        builder.template AddSystem<
            systems::controllers::PidController<T>>(Kp_v, Ki_v, Kd_v);

    auto plant_input_adder = builder.template AddSystem<
        systems::Adder<T>>(2, plant_input_port.size());

    const auto& plant_output_port =
        plant_->get_output_port(plant_state_port_index);

    builder.Connect(plant_output_port,
                    controller->get_input_port_estimated_state());
    builder.Connect(controller->get_output_port_control(),
                    plant_input_adder->get_input_port(0));
    builder.Connect(plant_input_adder->get_output_port(), plant_input_port);

    // Input port for ustar:
    ustar_port_ = builder.ExportInput(plant_input_adder->get_input_port(1));

    // Input port for xstar:
    xstar_port_ = builder.ExportInput(controller->get_input_port_desired_state());

    geometry_ids_ = builder.ExportOutput(plant->get_geometry_id_output_port());
    geometry_poses_ = builder.ExportOutput(plant->get_geometry_pose_output_port());


    // Output port for plant state:
    builder.ExportOutput(plant_output_port);
    builder.BuildInto(this);
  }

  const systems::InputPortDescriptor<T>& get_control_input_port() const {
    return this->get_input_port(ustar_port_);
  }

  const systems::InputPortDescriptor<T>& get_state_input_port() const {
    return this->get_input_port(xstar_port_);
  }

  const systems::OutputPort<T>& get_geometry_id_output_port() const {
    return this->get_output_port(geometry_ids_);
  }

  const systems::OutputPort<T>& get_geometry_pose_output_port() const {
    return this->get_output_port(geometry_poses_);
  }

 private:
  int ustar_port_{-1}, xstar_port_{-1};
  int geometry_ids_{-1}, geometry_poses_{-1};
  PendulumPlant<T>* plant_{nullptr};
};

std::tuple<std::unique_ptr<Trajectory>, std::unique_ptr<Trajectory>>
ComputeTrajectory(double mass, double length, double gravity) {
  PendulumPlant<double> pendulum(mass, length, gravity);
  auto context = pendulum.CreateDefaultContext();

  const int kNumTimeSamples = 21;
  const double kMinimumTimeStep = 0.2;
  const double kMaximumSampleTime = 0.5;
  systems::trajectory_optimization::DirectCollocation dircol(
      &pendulum, *context, kNumTimeSamples, kMinimumTimeStep,
      kMaximumSampleTime);

  dircol.AddEqualTimeIntervalsConstraints();

  // TODO(russt): Add this constraint to PendulumPlant and get it automatically
  // through DirectCollocation.
  const double kTorqueLimit = 3.0;  // N*m.
  const solvers::VectorXDecisionVariable& u = dircol.input();
  dircol.AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  dircol.AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  PendulumState<double> initial_state, final_state;
  initial_state.set_theta(0.0);
  initial_state.set_thetadot(0.0);
  final_state.set_theta(M_PI);
  final_state.set_thetadot(0.0);

  dircol.AddLinearConstraint(dircol.initial_state() ==
      initial_state.get_value());
  dircol.AddLinearConstraint(dircol.final_state() == final_state.get_value());

  const double R = 10;  // Cost on input "effort".
  dircol.AddRunningCost((R * u) * u);

  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state.get_value(), final_state.get_value()});
  dircol.SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);
  SolutionResult result = dircol.Solve();
  if (result != SolutionResult::kSolutionFound) {
    DRAKE_ABORT_MSG("Failed to solve optimization for the swing-up trajectory");
  }

  const PiecewisePolynomialTrajectory pp_traj =
      dircol.ReconstructInputTrajectory();
  const PiecewisePolynomialTrajectory pp_xtraj =
      dircol.ReconstructStateTrajectory();

  // strangely PiecewisePolynomialTrajectory is not copiable so we resource to
  // returning unique pointers.
  return std::make_tuple(pp_traj.Clone(), pp_xtraj.Clone());
};


int do_main() {
  // Define plant's parameters:
  const double mass = 1.0;      // [Kgr], about a pound.
  const double length = 0.5;    // [m]
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

  // Create a pendulum plant for simulation that registers geometry in
  // geometry_system for visualization.
  // We do not add it with the Builder yet since we will transfer ownership to
  // a PidControlledSystem below.
  auto pendulum = std::make_unique<PendulumPlant<double>>(
      mass, length, gravity, geometry_system);
  pendulum->set_name("pendulum");

  // Compute the optimum trajectory.
  std::unique_ptr<Trajectory> pp_ustar, pp_xstar;
  std::tie(pp_ustar, pp_xstar) = ComputeTrajectory(mass, length, gravity);

  // Create sources out of the computed trajectories:
  auto input_trajectory =
      builder.AddSystem<systems::TrajectorySource>(*pp_ustar);
  input_trajectory->set_name("optimal input trajectory");
  auto state_trajectory =
      builder.AddSystem<systems::TrajectorySource>(*pp_xstar);
  state_trajectory->set_name("optimal state trajectory");

  // Create the controller:
  // The choices of PidController constants here are fairly arbitrary,
  // but seem to effectively swing up the pendulum and hold it.
  const double Kp = 10;
  const double Ki = 0;
  const double Kd = 1;
  SourceId source_id = pendulum->get_source_id().value();
  auto pid_controlled_pendulum =
      builder.AddSystem<PidControlledSystem<double>>(
          std::move(pendulum),
          pendulum->get_input_port().get_index(),
          pendulum->get_state_output_port().get_index(),
          Kp, Ki, Kd);
  pid_controlled_pendulum->set_name("PID Controlled Pendulum");

  builder.Connect(input_trajectory->get_output_port(),
                  pid_controlled_pendulum->get_control_input_port());
  builder.Connect(state_trajectory->get_output_port(),
                  pid_controlled_pendulum->get_state_input_port());

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
      pid_controlled_pendulum->get_geometry_id_output_port(),
      geometry_system->get_source_frame_id_port(source_id));
  builder.Connect(
      pid_controlled_pendulum->get_geometry_pose_output_port(),
      geometry_system->get_source_pose_port(source_id));

  builder.Connect(geometry_system->get_pose_bundle_output_port(),
                  converter->get_input_port(0));
  builder.Connect(*converter, *publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(*geometry_system);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
#if 0
  systems::Context<double>& pendulum_context =
      diagram->GetMutableSubsystemContext(*pendulum,
                                          &simulator.get_mutable_context());

  // Set the state to be slightly off the fixed point:
  pendulum->SetAngle(&pendulum_context, M_PI + 0.5);
  pendulum->set_angular_velocity(&pendulum_context, 0.2);
#endif

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
  simulator.StepTo(pp_xstar->get_end_time());

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
