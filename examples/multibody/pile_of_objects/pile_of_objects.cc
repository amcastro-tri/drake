#include <memory>
#include <random>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/nice_type_name.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/random_rotation.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
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

DEFINE_double(simulation_time, 2.0, "Simulation duration in seconds");
DEFINE_double(
    mbp_time_step, 1.0E-3,
    "If mbp_time_step > 0, the fixed-time step period (in seconds) of discrete "
    "updates for the plant (modeled as a discrete system). "
    "If mbp_time_step = 0, the plant is modeled as a continuous system "
    "and no contact forces are displayed.  mbp_time_step must be >= 0.");
DEFINE_string(jacobian_scheme, "forward",
              "Valid Jacobian computation schemes are: "
              "'forward', 'central', or 'automatic'");
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
DEFINE_bool(only_collision_spheres, false,
            "Use only point contact with spheres");
DEFINE_bool(fixed_step, false, "Use fixed step integration. No error control.");

DEFINE_int32(objects_per_pile, 5, "Number of objects per pile.");
DEFINE_bool(visualize, true, "Whether to visualize (true) or not (false).");

using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using Eigen::Vector3d;
using Eigen::Translation3d;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;

// Parameters 
const double width(0.8);
const double length(0.8);

const RigidBody<double>& AddBox(const std::string& name,
                           const Vector3<double>& block_dimensions,
                           double mass, double friction,
                           const Vector4<double>& color,
                           bool only_sphere_collision,
                           MultibodyPlant<double>* plant) {
  // Ensure the block's dimensions are mass are positive.
  const double LBx = block_dimensions.x();
  const double LBy = block_dimensions.y();
  const double LBz = block_dimensions.z();

  // Describe body B's mass, center of mass, and inertia properties.
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBcm_B =
      UnitInertia<double>::SolidBox(LBx, LBy, LBz);
  const SpatialInertia<double> M_BBcm_B(mass, p_BoBcm_B, G_BBcm_B);

  // Create a rigid body B with the mass properties of a uniform solid block.
  const RigidBody<double>& box = plant->AddRigidBody(name, M_BBcm_B);

  // Box's visual.
  // The pose X_BG of block B's geometry frame G is an identity transform.
  const RigidTransform<double> X_BG;   // Identity transform.  
  plant->RegisterVisualGeometry(box, X_BG,
                                geometry::Box(LBx, LBy, LBz),
                                name + "_visual", color);

  // Box's collision geometry is a solid box.
  if (only_sphere_collision) {
    const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
    const double radius = LBz / 8;
    int i = 0;
    for (double x_sign : {-1.0, 0.0, 1.0}) {
      for (double y_sign : {-1.0, 0.0, 1.0}) {
        for (double z_sign : {-1.0, 0.0, 1.0}) {
          const std::string name_spherei =
              name + "_sphere" + std::to_string(++i) + "_collision";
          const double x = x_sign * LBx / 2;
          const double y = y_sign * LBy / 2;
          const double z = z_sign * LBz / 2;
          const Vector3<double> p_BoSpherei_B(x, y, z);
          const RigidTransform<double> X_BSpherei(p_BoSpherei_B);
          plant->RegisterCollisionGeometry(
              box, X_BSpherei, geometry::Sphere(radius), name_spherei,
              CoulombFriction<double>(friction, friction));
          plant->RegisterVisualGeometry(
              box, X_BSpherei, geometry::Sphere(radius), name_spherei, red);
        }  // z
      }  // y
    }  // x
  } else {
    plant->RegisterCollisionGeometry(
        box, X_BG, geometry::Box(LBx, LBy, LBz), name + "_collision",
        CoulombFriction<double>(friction, friction));
  }
  return box;                                     
}

void AddSink(MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  // Parameters for the sink.  
  //const double length = 1.0;
  //const double width = 0.8;  
  const double height = 0.4;
  const double wall_thickness = 0.04;
  const double wall_mass = 1.0; 
  const double friction_coefficient = 0.5; 
  const Vector4<double> light_blue(0.5, 0.8, 1.0, 0.3);

  auto add_wall = [&](const std::string& name, const Vector3d& dimensions,
                      const RigidTransformd& X_WB) -> const RigidBody<double>& {
    const auto& wall =
        AddBox(name, dimensions, wall_mass,
               friction_coefficient, light_blue, false, plant);
    plant->WeldFrames(plant->world_frame(), wall.body_frame(), X_WB);
    return wall;
  };

  const Vector3d bottom_dimensions(length, width, wall_thickness);
  const Vector3d side_wall_dimensions(height, width, wall_thickness);
  const Vector3d back_front_wall_dimensions(length, wall_thickness, height);

  add_wall("sink_bottom", bottom_dimensions,
           Translation3d(0, 0, -wall_thickness / 2.0));
  add_wall("sink_right", side_wall_dimensions,
           RigidTransformd(RotationMatrixd::MakeYRotation(M_PI_2),
                           Vector3d(length / 2.0, 0.0, height / 2.0)));
  add_wall("sink_left",side_wall_dimensions,
           RigidTransformd(RotationMatrixd::MakeYRotation(M_PI_2),
                           Vector3d(-length / 2.0, 0.0, height / 2.0)));
  add_wall("sink_back", back_front_wall_dimensions,
           Translation3d(0.0, width / 2, height / 2));
  add_wall("sink_front", back_front_wall_dimensions,
           Translation3d(0.0, -width / 2, height / 2));
}

const RigidBody<double>& AddSphere(const std::string& name, const double radius,
                                   double mass, double friction,
                                   const Vector4<double>& color,
                                   MultibodyPlant<double>* plant) {
  const UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  const SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody(name, M_Bcm);

  // Add collision geometry.
  const RigidTransformd X_BS = RigidTransformd::Identity();
  plant->RegisterCollisionGeometry(ball, X_BS, geometry::Sphere(radius),
                                   name + "_collision",
                                   CoulombFriction<double>(friction, friction));

  // Add visual geometry.
  plant->RegisterVisualGeometry(ball, X_BS, geometry::Sphere(radius),
                                name + "_visual", color);

  // We add a few spots so that we can appreciate the sphere's
  // rotation, colored on red, green, blue according to the body's axes.
  const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
  const Vector4<double> green(0.0, 1.0, 0.0, 1.0);
  const Vector4<double> blue(0.0, 0.0, 1.0, 1.0);
  const double visual_radius = 0.2 * radius;
  const geometry::Cylinder spot(visual_radius, visual_radius);
  // N.B. We do not place the cylinder's cap exactly on the sphere surface to
  // avoid visualization artifacts when the surfaces are kissing.
  const double radial_offset = radius - 0.45 * visual_radius;
  auto spot_pose = [](const Vector3<double>& position) {
    // The cylinder's z-axis is defined as the normalized vector from the
    // sphere's origin to the cylinder's center `position`.
    const Vector3<double> axis = position.normalized();
    return RigidTransformd(
        Eigen::Quaterniond::FromTwoVectors(Vector3<double>::UnitZ(), axis),
        position);
  };
  plant->RegisterVisualGeometry(ball, spot_pose({radial_offset, 0., 0.}), spot,
                                name + "_x+", red);
  plant->RegisterVisualGeometry(ball, spot_pose({-radial_offset, 0., 0.}), spot,
                                name + "_x-", red);
  plant->RegisterVisualGeometry(ball, spot_pose({0., radial_offset, 0.}), spot,
                                name + "_y+", green);
  plant->RegisterVisualGeometry(ball, spot_pose({0., -radial_offset, 0.}), spot,
                                name + "_y-", green);
  plant->RegisterVisualGeometry(ball, spot_pose({0., 0., radial_offset}), spot,
                                name + "_z+", blue);
  plant->RegisterVisualGeometry(ball, spot_pose({0., 0., -radial_offset}), spot,
                                name + "_z-", blue);                                      
  return ball;                                
}

std::vector<BodyIndex> AddObjects(MultibodyPlant<double>* plant) {
  const double radius = 0.05;
  const double mass = 0.2;
  const double friction = 0.5;
  const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  const Vector4<double> purple(204.0/255, 0.0, 204.0/255, 1.0);
  const Vector4<double> green(0, 153.0/255, 0, 1.0);
  const Vector4<double> cyan(51/255, 1.0, 1.0, 1.0);
  const Vector4<double> pink(1.0,204.0/255, 204.0/255, 1.0);
  std::vector<Vector4<double>> colors;
  colors.push_back(orange);
  colors.push_back(purple);
  colors.push_back(green);
  colors.push_back(cyan);
  colors.push_back(pink);

  const int seed = 41;
  std::mt19937 generator(seed);
  std::uniform_int_distribution<int> distribution(0,1);

  auto roll_shape = [&]() {
      return distribution(generator);
  };

  const int num_objects = FLAGS_objects_per_pile;
  const int num_bodies = plant->num_bodies();

  std::vector<BodyIndex> bodies;
  for (int i = 1; i <= num_objects; ++i) {
    const auto& color = colors[(i - 1) % colors.size()];
    const std::string name = "object" + std::to_string(i+num_bodies);
    switch (roll_shape()) {
      case 0:
        bodies.push_back(
            AddSphere(name, radius, mass, friction, color, plant).index());
        break;
      case 1:
        const Vector3d size = 2 * radius * Vector3d::Ones();
        bodies.push_back(AddBox(name, size, mass, friction, color,
                                FLAGS_only_collision_spheres, plant)
                             .index());
        break;
    }
  }

  return bodies;
}

void SetInitialConditions(const MultibodyPlant<double>& plant,
                          const Vector3d& offset, 
                          const std::vector<BodyIndex>& bodies,
                          systems::Context<double>* plant_context) {
  const double delta_z = 0.15;  // assume objects have a BB of about 10 cm.

  const int seed = 41;
  std::mt19937 generator(seed);

  double z = delta_z/2;
  for (auto body_index : bodies) {
    const auto& body = plant.get_body(body_index);
    if (body.is_floating()) {
      const RotationMatrixd R_WB =
          math::UniformlyRandomRotationMatrix<double>(&generator);
      const Vector3d p_WB = offset + Vector3d(0.0, 0.0, z);

      plant.SetFreeBodyPose(plant_context, body, RigidTransformd(R_WB, p_WB));
      z += delta_z;
    }
  }
}

int do_main() {
  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      AddMultibodyPlantSceneGraph(&builder, FLAGS_mbp_time_step);

  AddSink(&plant);

  //const double radius = 0.05;
  //const double mass = 0.2;
  //const double friction = 0.5;
  //const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);

  //AddSphere("sphere", radius, mass, friction, orange, &plant);
  auto pile1 = AddObjects(&plant);
  auto pile2 = AddObjects(&plant);
  auto pile3 = AddObjects(&plant);
  auto pile4 = AddObjects(&plant);

  plant.Finalize();
  plant.set_penetration_allowance(FLAGS_penetration_allowance);

  // Set the speed tolerance (m/s) for the underlying Stribeck friction model
  // (the allowable drift speed during stiction).  For two points in contact,
  // this is the maximum sliding speed for the points to be regarded as
  // stationary relative to each other (so that static friction is used).
  plant.set_stiction_tolerance(FLAGS_stiction_tolerance);

  fmt::print("Num positions: {:d}\n", plant.num_positions());
  fmt::print("Num velocities: {:d}\n", plant.num_velocities());

  // Publish contact results for visualization.
  if (FLAGS_visualize) {
    ConnectContactResultsToDrakeVisualizer(&builder, plant);
    geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  }
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

  SetInitialConditions(plant, Vector3d(length / 4, width / 4, 0), pile1,
                       &plant_context);
  SetInitialConditions(plant, Vector3d(-length / 4, width / 4, 0), pile2,
                       &plant_context);
  SetInitialConditions(plant, Vector3d(-length / 4, -width / 4, 0), pile3,
                       &plant_context);
  SetInitialConditions(plant, Vector3d(length / 4, -width / 4, 0), pile4,
                       &plant_context);

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

  simulator->AdvanceTo(FLAGS_simulation_time);

  if (plant.is_discrete()) {
    fmt::print("Used time stepping with dt={}\n", plant.time_step());
  }

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
