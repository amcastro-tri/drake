#include <memory>
#include <utility>
#include <chrono>


#include <gflags/gflags.h>

#include "drake/common/nice_type_name.h"
#include "drake/examples/integration_with_events/bouncing_ball_plant.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
//#include "drake/systems/lcm/lcm_publisher_system.h"

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

DEFINE_string(jacobian_scheme, "forward",
              "Valid Jacobian computation schemes are: "
              "'forward', 'central', or 'automatic'");
DEFINE_bool(fixed_step, false, "Use fixed step integration. No error control.");
DEFINE_bool(with_normal_event, true, "There is event for zero distance.");

namespace drake {
namespace examples {
namespace scene_graph {
namespace bouncing_ball {
namespace {

class Timer {
  public:
    Timer() {
      start();
    }

    void start() {
      start_ = clock::now();
    }

    double end() {
      const clock::time_point end = clock::now();
      return std::chrono::duration<double>(end - start_).count();
    }
  private:
    using clock = std::chrono::steady_clock;
    clock::time_point start_;
};  

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::ConnectDrakeVisualizer;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::GeometryId;
using geometry::HalfSpace;
using geometry::IllustrationProperties;
using geometry::PerceptionProperties;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using geometry::SourceId;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RotationMatrixd;
using systems::InputPort;
using std::make_unique;

int do_main() {
  systems::DiagramBuilder<double> builder;
  auto scene_graph = builder.AddSystem<SceneGraph<double>>();
  scene_graph->set_name("scene_graph");

  const SourceId ball_source_id1 = scene_graph->RegisterSource("ball1");
  auto bouncing_ball1 = builder.AddSystem<BouncingBallPlant>(
      ball_source_id1, scene_graph, Vector2<double>(0.25, 0.25),
      FLAGS_with_normal_event);
  bouncing_ball1->set_name("BouncingBall1");

  const SourceId global_source = scene_graph->RegisterSource("anchored");
  // Add a "ground" halfspace. Define the pose of the half space (H) in the
  // world from its normal (Hz_W) and a point on the plane (p_WH). In this case,
  // X_WH will be the identity.
  Vector3<double> Hz_W(0, 0, 1);
  Vector3<double> p_WHo_W(0, 0, 0);
  const GeometryId ground_id = scene_graph->RegisterAnchoredGeometry(
      global_source,
      make_unique<GeometryInstance>(HalfSpace::MakePose(Hz_W, p_WHo_W),
                                    make_unique<HalfSpace>(), "ground"));
  scene_graph->AssignRole(global_source, ground_id, ProximityProperties());
  scene_graph->AssignRole(global_source, ground_id, IllustrationProperties());

  builder.Connect(bouncing_ball1->get_geometry_pose_output_port(),
                  scene_graph->get_source_pose_port(ball_source_id1));
  builder.Connect(scene_graph->get_query_output_port(),
                  bouncing_ball1->get_geometry_query_input_port());

  DrakeLcm lcm;
  ConnectDrakeVisualizer(&builder, *scene_graph, &lcm);

  auto diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());

  auto init_ball = [&](BouncingBallPlant<double>* system, double z,
                       double zdot) {
    systems::Context<double>& ball_context =
        diagram->GetMutableSubsystemContext(*system,
                                            diagram_context.get());
    system->set_z(&ball_context, z);
    system->set_zdot(&ball_context, zdot);
  };
  init_ball(bouncing_ball1, 0.3, 0.);

  auto simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  systems::IntegratorBase<double>& integrator =
      simulator->get_mutable_integrator();
  auto* implicit_integrator =
      dynamic_cast<systems::ImplicitIntegrator<double>*>(&integrator);
  if (implicit_integrator) {
    if (FLAGS_jacobian_scheme == "forward") {
      implicit_integrator->set_jacobian_computation_scheme(
          systems::ImplicitIntegrator<
              double>::JacobianComputationScheme::kForwardDifference);
    } else if (FLAGS_jacobian_scheme == "central") {
      implicit_integrator->set_jacobian_computation_scheme(
          systems::ImplicitIntegrator<
              double>::JacobianComputationScheme::kCentralDifference);
    } else if (FLAGS_jacobian_scheme == "automatic") {
      implicit_integrator->set_jacobian_computation_scheme(
          systems::ImplicitIntegrator<
              double>::JacobianComputationScheme::kAutomatic);
    } else {
      throw std::runtime_error("Invalid Jacobian computation scheme");
    }
  }
  if (integrator.supports_error_estimation())
    integrator.set_fixed_step_mode(FLAGS_fixed_step);

  Timer timer;
  simulator->AdvanceTo(FLAGS_simulation_time);
  const double sim_time = timer.end();

  fmt::print("Simulation run in {:10.6g} seconds.\n", sim_time);

  fmt::print("\n### SIMULATOR STATISTICS ###\n");
  fmt::print("Number of steps = {:d}\n", simulator->get_num_steps_taken());
  fmt::print("Number of discrete updates = {:d}\n", simulator->get_num_discrete_updates());
  fmt::print("Number of publishes = {:d}\n", simulator->get_num_publishes());

  fmt::print("Stats for integrator {}:\n",
             drake::NiceTypeName::Get(integrator));
  fmt::print("\n### GENERAL INTEGRATION STATISTICS ###\n");
  fmt::print("Number of time steps taken = {:d}\n",
             integrator.get_num_steps_taken());
  fmt::print("Number of derivative evaluations = {:d}\n",
             integrator.get_num_derivative_evaluations());
  // if (integrator.supports_error_estimation()) {
  fmt::print("\n### ERROR CONTROL STATISTICS ###\n");
  fmt::print("Initial time step taken = {:10.6g} s\n",
             integrator.get_actual_initial_step_size_taken());
  fmt::print("Largest time step taken = {:10.6g} s\n",
             integrator.get_largest_step_size_taken());
  fmt::print("Smallest adapted step size = {:10.6g} s\n",
             integrator.get_smallest_adapted_step_size_taken());
  fmt::print("Number of steps shrunk due to error control = {:d}\n",
             integrator.get_num_step_shrinkages_from_error_control());
  fmt::print("Number of step failues = {:d}\n",
             integrator.get_num_substep_failures());
  fmt::print("Number of steps shrunk due to step failues = {:d}\n",
             integrator.get_num_step_shrinkages_from_substep_failures());

  //}

  if (implicit_integrator) {
    fmt::print("\n### IMPLICIT INTEGRATION STATISTICS ###\n");
    fmt::print("Number of Jacobian evaluations = {:d}\n",
               implicit_integrator->get_num_jacobian_evaluations());
    fmt::print("Number of Jacobian factorizations = {:d}\n",
               implicit_integrator->get_num_iteration_matrix_factorizations());
    fmt::print("Number of derivative evaluations for...\n");
    fmt::print(
        "  Jacobian = {:d}\n",
        implicit_integrator->get_num_derivative_evaluations_for_jacobian());
    fmt::print(
        "  Error estimate = {:d}\n",
        implicit_integrator->get_num_error_estimator_derivative_evaluations());
    fmt::print(
        "  Error estimate Jacobian = {:d}\n",
        implicit_integrator
            ->get_num_error_estimator_derivative_evaluations_for_jacobian());
  }

  fmt::print("Integrator time[s] #steps #xdot_evals #shrunk_by_errc #shrunk_by_fails");
  //if (implicit_integrator) {
  fmt::print(" #error_est_xdot_evals #jac_evals #matrx_facts");
  //}
  fmt::print("\n");
  fmt::print("{} ", FLAGS_integration_scheme);
  fmt::print("{:10.6g} ", sim_time);
  fmt::print("{:d} ", integrator.get_num_steps_taken());
  fmt::print("{:d} ", integrator.get_num_derivative_evaluations());
  fmt::print("{:d} ", integrator.get_num_step_shrinkages_from_error_control());
  fmt::print("{:d} ", integrator.get_num_step_shrinkages_from_substep_failures());
  if (implicit_integrator) {
    fmt::print(
        "{:d} ",
        implicit_integrator->get_num_error_estimator_derivative_evaluations());
    fmt::print("{:d} ", implicit_integrator->get_num_jacobian_evaluations());
    fmt::print("{:d} ", implicit_integrator->get_num_iteration_matrix_factorizations());
  } else {
    fmt::print("-1 -1 -1 ");
  }
  fmt::print("\n");

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::scene_graph::bouncing_ball::do_main();
}
