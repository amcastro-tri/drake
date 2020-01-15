#include <memory>

#include <gflags/gflags.h>

#include "drake/common/nice_type_name.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace examples {
namespace inclined_plane_with_body {
namespace {

// This simulates the motion of a rigid body B (e.g., a sphere or a block) on an
// inclined plane A (which may be an infinite half-space or a finite box).
// TODO(Mitiguy) Consider an option to configure body B to be in contact with
// inclined plane A, or allow for user-defined initial configuration/motion.
//
// Information on how to build, run, and visualize this example and how to use
// command-line arguments is in the accompanying file README.md.
DEFINE_double(simulation_time, 2.0, "Simulation duration in seconds");
DEFINE_double(
    mbp_time_step, 1.0E-3,
    "If mbp_time_step > 0, the fixed-time step period (in seconds) of discrete "
    "updates for the plant (modeled as a discrete system). "
    "If mbp_time_step = 0, the plant is modeled as a continuous system "
    "and no contact forces are displayed.  mbp_time_step must be >= 0.");
DEFINE_double(penetration_allowance, 1.0E-5, "Allowable penetration (meters).");
DEFINE_double(stiction_tolerance, 1.0E-5,
              "Allowable drift speed during stiction (m/s).");
DEFINE_double(inclined_plane_angle_degrees, 15.0,
              "Inclined plane angle (degrees), i.e., angle from Wx to Ax.");
DEFINE_double(inclined_plane_coef_static_friction, 0.3,
              "Inclined plane's coefficient of static friction (no units).");
DEFINE_double(inclined_plane_coef_kinetic_friction, 0.3,
              "Inclined plane's coefficient of kinetic friction (no units).  "
              "When mbp_time_step > 0, this value is ignored.  Only the "
              "coefficient of static friction is used in fixed-time step.");
DEFINE_double(bodyB_coef_static_friction, 0.3,
              "Body B's coefficient of static friction (no units).");
DEFINE_double(bodyB_coef_kinetic_friction, 0.3,
              "Body B's coefficient of kinetic friction (no units).  "
              "When mbp_time_step > 0, this value is ignored.  Only the "
              "coefficient of static friction is used in fixed-time step.");
DEFINE_bool(is_inclined_plane_half_space, true,
            "Is inclined plane a half-space (true) or box (false).");
DEFINE_string(bodyB_type, "sphere", "Valid body types are "
              "'sphere', 'block', or 'block_with_4Spheres'");

using drake::multibody::MultibodyPlant;

int do_main() {
  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;
  auto pair = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(FLAGS_mbp_time_step));
  MultibodyPlant<double>& plant = pair.plant;

  // Set constants that are relevant whether body B is a sphere or block.
  const double massB = 0.1;       // Body B's mass (kg).
  const double gravity = 9.8;     // Earth's gravitational acceleration (m/s^2).
  const double inclined_plane_angle =
      FLAGS_inclined_plane_angle_degrees / 180 * M_PI;

  // Information on how coefficients of friction are used in the file README.md
  // (which is in the folder associated with this example).
  const drake::multibody::CoulombFriction<double> coef_friction_bodyB(
      FLAGS_bodyB_coef_static_friction, FLAGS_bodyB_coef_kinetic_friction);
  const drake::multibody::CoulombFriction<double> coef_friction_inclined_plane(
      FLAGS_inclined_plane_coef_static_friction,
      FLAGS_inclined_plane_coef_kinetic_friction);

  if (FLAGS_bodyB_type == "sphere") {
    const double radiusB = 0.04;      // B's radius when modeled as a sphere.
    const double LAx = 20 * radiusB;  // Inclined plane length in Ax direction.
    const double LAy = 10 * radiusB;  // Inclined plane length in Ay direction.
    const double LAz = radiusB;       // Inclined plane length in Az direction.
    const Vector3<double> LAxyz(LAx, LAy, LAz);
    const std::optional<Vector3<double>> inclined_plane_dimensions =
        FLAGS_is_inclined_plane_half_space ? std::nullopt
            : std::optional<Vector3<double>>(LAxyz);
    benchmarks::inclined_plane::AddInclinedPlaneWithSphereToPlant(
        gravity, inclined_plane_angle, inclined_plane_dimensions,
        coef_friction_inclined_plane, coef_friction_bodyB,
        massB, radiusB, &plant);
  } else if (FLAGS_bodyB_type == "block" ||
             FLAGS_bodyB_type == "block_with_4Spheres") {
    // B's contacting surface can be modeled with 4 spheres or a single box.
    const bool is_bodyB_block_with_4Spheres =
        (FLAGS_bodyB_type == "block_with_4Spheres");
    const double LBx = 0.4;      // Block B's length in Bx-direction (meters).
    const double LBy = 0.2;      // Block B's length in By-direction (meters).
    const double LBz = 0.04;     // Block B's length in Bz-direction (meters).
    const double LAx = 8 * LBx;  // Inclined plane A's length in Ax direction.
    const double LAy = 8 * LBy;  // Inclined plane A's length in Ay direction.
    const double LAz = 0.04;     // Inclined plane A's length in Az direction.
    const Vector3<double> block_dimensions(LBx, LBy, LBz);
    const Vector3<double> LAxyz(LAx, LAy, LAz);
    const std::optional<Vector3<double>> inclined_plane_dimensions =
        FLAGS_is_inclined_plane_half_space ? std::nullopt
            : std::optional<Vector3<double>>(LAxyz);
    benchmarks::inclined_plane::AddInclinedPlaneWithBlockToPlant(
        gravity, inclined_plane_angle, inclined_plane_dimensions,
        coef_friction_inclined_plane, coef_friction_bodyB,
        massB, block_dimensions,
        is_bodyB_block_with_4Spheres, &plant);
  } else {
    std::cerr << "Invalid body_type '" << FLAGS_bodyB_type
              << "' (note that types are case sensitive)." << std::endl;
    return -1;
  }

  plant.Finalize();
  plant.set_penetration_allowance(FLAGS_penetration_allowance);

  // Set the speed tolerance (m/s) for the underlying Stribeck friction model
  // (the allowable drift speed during stiction).  For two points in contact,
  // this is the maximum sliding speed for the points to be regarded as
  // stationary relative to each other (so that static friction is used).
  plant.set_stiction_tolerance(FLAGS_stiction_tolerance);

  // Do a reality check that body B is a free-flying rigid body.
  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Publish contact results for visualization.
  ConnectContactResultsToDrakeVisualizer(&builder, plant);

  geometry::ConnectDrakeVisualizer(&builder, pair.scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // In the plant's default context, we assume the state of body B in world W is
  // such that X_WB is an identity transform and B's spatial velocity is zero.
  plant.SetDefaultContext(&plant_context);

  // Overwrite B's default initial position so it is somewhere above the
  // inclined plane provided `0 < inclined_plane_angle < 40`.
  const drake::multibody::Body<double>& bodyB = plant.GetBodyByName("BodyB");
  const Vector3<double> p_WoBo_W(-1.0, 0, 1.2);
  const math::RigidTransform<double> X_WB(p_WoBo_W);
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, bodyB, X_WB);

  auto simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));
  simulator->AdvanceTo(FLAGS_simulation_time);

  if (plant.is_discrete()) {
    fmt::print("Used time stepping with dt={}\n", plant.time_step());
  }

  const systems::IntegratorBase<double>& integrator =
      simulator->get_integrator();
  fmt::print("Stats for integrator {}:\n",
             drake::NiceTypeName::Get(integrator));
  fmt::print("\n### GENERAL INTEGRATION STATISTICS ###\n");
  fmt::print("Number of time steps taken = {:d}\n",
             integrator.get_num_steps_taken());
  fmt::print("Number of derivative evaluations = {:d}\n",
             integrator.get_num_derivative_evaluations());
  if (!integrator.get_fixed_step_mode()) {
    fmt::print("\n### ERROR CONTROL STATISTICS ###\n");
    fmt::print("Initial time step taken = {:10.6g} s\n",
               integrator.get_actual_initial_step_size_taken());
    fmt::print("Largest time step taken = {:10.6g} s\n",
               integrator.get_largest_step_size_taken());
    fmt::print("Smallest adapted step size = {:10.6g} s\n",
               integrator.get_smallest_adapted_step_size_taken());
    fmt::print("Number of steps shrunk due to error control = {:d}\n",
               integrator.get_num_step_shrinkages_from_error_control());
  }

  const auto* implicit_integrator =
      dynamic_cast<const systems::ImplicitIntegrator<double>*>(&integrator);
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

  return 0;
}

}  // namespace
}  // namespace inclined_plane_with_body
}  // namespace examples
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "\nSimulation of a body (e.g., sphere or block) on an inclined plane."
      "\nThe type of body is user-selected and may slip or stick (roll)."
      "\nInformation on how to build, run, and visualize this example and how"
      "\nto use command-line arguments is in the file README.md"
      "\n(which is in the folder associated with this example).\n");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::examples::inclined_plane_with_body::do_main();
}
