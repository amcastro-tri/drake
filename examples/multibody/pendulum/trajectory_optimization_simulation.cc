#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

using drake::geometry::SceneGraph;
using drake::geometry::SourceId;
using drake::lcm::DrakeLcm;
using drake::solvers::SolutionResult;
using drake::trajectories::PiecewisePolynomial;
using drake::multibody::benchmarks::pendulum::MakePendulumPlant;
using drake::multibody::benchmarks::pendulum::PendulumParameters;
using drake::multibody::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::systems::trajectory_optimization::DirectCollocation;

namespace drake {
namespace examples {
namespace pendulum {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

struct PendulumTrajOptimizationSolution {
  // DirectCollocation makes reference to the plant. So we keep it alive here.
  std::unique_ptr<MultibodyPlant<double>> plant;
  std::unique_ptr<DirectCollocation> dircol;
  drake::solvers::MathematicalProgramResult result;
  VectorX<double> initial_state;
  VectorX<double> final_state;
};

auto SetUpAndSolveTrajOptimization(
    PendulumParameters parameters) {
  auto pendulum = MakePendulumPlant(parameters);   
  const RevoluteJoint<double>& pin =
      pendulum->GetJointByName<RevoluteJoint>(parameters.pin_joint_name());

  // Make initial/final state vectors.
  auto context = pendulum->CreateDefaultContext();
  pin.set_angle(context.get(), 0.0);
  pin.set_angular_rate(context.get(), 0.0);
  const auto initial_state =
      context->get_continuous_state_vector().CopyToVector();

  pin.set_angle(context.get(), M_PI);
  pin.set_angular_rate(context.get(), 0.0);
  const auto final_state =
      context->get_continuous_state_vector().CopyToVector();

  const int kNumTimeSamples = 21;
  const double kMinimumTimeStep = 0.2;
  const double kMaximumTimeStep = 0.5;
  // Reuse context here, though the values do not have any effect.
  auto dircol =
      std::make_unique<systems::trajectory_optimization::DirectCollocation>(
          pendulum.get(), *context, kNumTimeSamples, kMinimumTimeStep,
          kMaximumTimeStep, pendulum->get_actuation_input_port().get_index());
  dircol->AddEqualTimeIntervalsConstraints();

  // TODO(russt): Add this constraint to PendulumPlant and get it automatically
  // through DirectCollocation.
  const double kTorqueLimit = 3.0;  // N*m.
  const solvers::VectorXDecisionVariable& u = dircol->input();
  DRAKE_DEMAND(u.size() == 1);
  dircol->AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  dircol->AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  dircol->AddLinearConstraint(dircol->initial_state() == initial_state);
  dircol->AddLinearConstraint(dircol->final_state() == final_state);

  const double R = 10;  // Cost on input "effort".
  dircol->AddRunningCost((R * u) * u);

  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {initial_state, final_state});
  dircol->SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);
  const auto result = solvers::Solve(*dircol);
  if (!result.is_success()) {
    throw std::runtime_error(
        "Failed to solve optimization for the swing-up trajectory");
  }

  PendulumTrajOptimizationSolution solution;
  solution.plant = std::move(pendulum);
  solution.dircol = std::move(dircol);
  solution.result = result;
  solution.initial_state = initial_state;
  solution.final_state = final_state;

  return solution;
}

int DoMain() {
  // The model's parameters:
  PendulumParameters parameters;

  auto traj_opt_solution = SetUpAndSolveTrajOptimization(parameters);
  auto& dircol = *traj_opt_solution.dircol;
  auto& result = traj_opt_solution.result;

  // Set up diagram for simulation.  
  systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double>& pendulum =
      *builder.AddSystem(MakePendulumPlant(parameters, &scene_graph));
  pendulum.set_name("pendulum");

  const PiecewisePolynomial<double> pp_traj =
      dircol.ReconstructInputTrajectory(result);
  const PiecewisePolynomial<double> pp_xtraj =
      dircol.ReconstructStateTrajectory(result);
  auto input_trajectory = builder.AddSystem<systems::TrajectorySource>(pp_traj);
  input_trajectory->set_name("input trajectory");
  auto state_trajectory =
      builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  state_trajectory->set_name("state trajectory");

  // The choices of PidController constants here are fairly arbitrary,
  // but seem to effectively swing up the pendulum and hold it.
  const double Kp = 10.0;
  const double Ki = 0.0;
  const double Kd = 1.0;
  auto connect_result =
      systems::controllers::PidControlledSystem<double>::ConnectController(
          pendulum.get_actuation_input_port(), pendulum.get_state_output_port(),
          Vector1d{Kp}, Vector1d{Ki}, Vector1d{Kd}, &builder);
  builder.Connect(input_trajectory->get_output_port(),
                  connect_result.control_input_port);
  builder.Connect(state_trajectory->get_output_port(),
                  connect_result.state_input_port);

  // Connect visualization.
  builder.Connect(
      pendulum.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(pendulum.get_source_id().value()));
  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(pp_xtraj.end_time());

  const auto& pendulum_state = pendulum.CopyContinuousStateVector(
      diagram->GetSubsystemContext(pendulum, simulator.get_context()));

  if (!is_approx_equal_abstol(pendulum_state, traj_opt_solution.final_state,
                              1e-3)) {
    throw std::runtime_error("Did not reach trajectory target.");
  }

  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::pendulum::DoMain();
}
