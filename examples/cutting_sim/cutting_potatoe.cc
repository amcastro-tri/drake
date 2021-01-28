#include <drake/systems/controllers/pid_controlled_system.h>
#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/tree/cut_magnet.h"
#include "drake/multibody/tree/particle_joint.h"
#include "drake/multibody/tree/linear_spring_damper.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

DEFINE_double(simulation_time, 0.5, "How long to simulate the pendulum");
DEFINE_double(mbp_dt, 0.0,
              "Simulation time step used for integrator.");

DEFINE_double(Kp_, 30.0, "Kp");
DEFINE_double(Ki_, 0.0, "Ki");
DEFINE_double(Kd_, 0.0, "Kd");

DEFINE_double(force_scale_, 20.0, "force scale for magnet");
DEFINE_double(cutoff_length, 1e-3,
              "Distance threshold to turn off magnets");
DEFINE_int32(grid_i, 3, "Number of spheres along x dimension of grid.");
DEFINE_int32(grid_j, 8, "Number of spheres along y dimension of grid.");
DEFINE_int32(grid_k, 3, "Number of spheres along z dimension of grid.");
DEFINE_double(sphere_radius, 0.01, "Sphere radius.");
DEFINE_double(particle_density, 1000.0, "Soft particles density, [kg/m³].");

// I estimated the elastic modulus from measurements in [Cheng et al., 2005].
// From figured 3 a fit a line between zero and ~2000 Pa at ~20% strain. This
// leads to a modulus of E = 10000 Pa.
//
// Cheng, Yongqiang, Naoto Shimizu, and Toshinori Kimura. "The viscoelastic
// properties of soybean curd (tofu) as affected by soymilk concentration and
// type of coagulant." International journal of food science & technology 40, no
// 4 (2005): 385-390.
DEFINE_double(elastic_modulus, 10000.0,
              "Linear spring model stiffness, [N/m²].");
DEFINE_double(linear_damping_ratio, 1.0,
              "Linear spring model damping ratio, [-].");
DEFINE_double(linear_damping, 0.0, "Linear spring model damping, [Ns/m].");
DEFINE_double(dynamic_friction, 0.2, "Friction coefficient, [-].");
DEFINE_double(grid_x, 0.26, "Initial x position of the grid's center.");
DEFINE_double(grid_y, 0, "Initial y position of the grid's center.");
DEFINE_double(grid_z, 0.76, "Initial z position of the grid's center.");

DEFINE_bool(use_cube, false,
            "If defined, elements are cubes (with measure equal to 2R).");
DEFINE_double(vis_rate, 30.0,
              "Frequency at which visualization messages are sent (in Hz");
DEFINE_bool(simple_table, false, "If defined the 'table' is a half space");
DEFINE_bool(forces_viz, false, "If true, visualize contact forces.");

namespace drake {
namespace examples {
namespace cutting_potatoe {
namespace {

using drake::multibody::Body;
using drake::multibody::CutMagnet;
using drake::multibody::LinearSpringDamper;
using drake::multibody::SpatialInertia;
using drake::multibody::ParticleJoint;
using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::PenetrationAsPointPair;
using geometry::PerceptionProperties;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::Sphere;
using geometry::render::RenderLabel;
using math::RigidTransform;
using math::RigidTransformd;
using std::make_unique;
using systems::Context;
using systems::MakeSimulatorFromGflags;
using systems::PrintSimulatorStatistics;
using systems::Simulator;

static const char* const kDoublePendulumSdfPath =
    "drake/examples/cutting_sim/models/guillotine.sdf";

std::map<std::tuple<int, int, int>, multibody::BodyIndex> make_grid(
    double mass, const Vector4<double>& color,
    multibody::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  const multibody::CoulombFriction<double> sphere_friction(
      FLAGS_dynamic_friction, FLAGS_dynamic_friction);

  const double sphere_ixyz = .4 * FLAGS_sphere_radius * FLAGS_sphere_radius;

  std::map<std::tuple<int, int, int>, multibody::BodyIndex> bodies;

  // For the set of geometries for all particles in the grid.
  drake::geometry::GeometrySet particles;

  // Register a regular grid of
  for (int i = 0; i < FLAGS_grid_i; ++i) {
    for (int j = 0; j < FLAGS_grid_j; ++j) {
      for (int k = 0; k < FLAGS_grid_k; ++k) {
        const std::tuple<int, int, int> index = std::make_tuple(i, j, k);

        const std::string name = fmt::format("sphere_{}_{}_{}", i, j, k);

        const Body<double>& sphere = plant->AddRigidBody(
            name, multibody::default_model_instance(),
            SpatialInertia<double>(mass, Vector3<double>::Zero(),
                                   multibody::UnitInertia<double>(
                                       sphere_ixyz, sphere_ixyz, sphere_ixyz)));

        plant->AddJoint(std::make_unique<ParticleJoint<double>>(
            "joint_" + name, plant->world_frame(), sphere.body_frame()));

        if (FLAGS_use_cube) {
          const geometry::Box box =
              geometry::Box::MakeCube(FLAGS_sphere_radius * 1.95);
          plant->RegisterVisualGeometry(sphere, RigidTransform<double>(), box,
                                        fmt::format("{}_visual", name), color);

          plant->RegisterCollisionGeometry(
              sphere, RigidTransform<double>(), box,
              fmt::format("{}_collision", name), sphere_friction);
        } else {
          plant->RegisterVisualGeometry(sphere, RigidTransform<double>(),
                                        geometry::Sphere(FLAGS_sphere_radius),
                                        fmt::format("{}_visual", name), color);

          plant->RegisterCollisionGeometry(
              sphere, RigidTransform<double>(),
              geometry::Sphere(FLAGS_sphere_radius),
              fmt::format("{}_collision", name), sphere_friction);
        }
        particles.Add(plant->GetCollisionGeometriesForBody(sphere));
        bodies[index] = sphere.index();
      }
    }
  }

  // Exclude collisions between spheres. Spheres only interact among them
  // through force elements.  
  scene_graph->ExcludeCollisionsWithin(particles);

  // Add force elements between neighboring spheres.
  // The "origins" of the magnets lie at the center of each element: point
  // P on element A, and point Q on element B.
  const Vector3<double> p_AP_{0, 0, 0};
  const Vector3<double> p_BQ_{0, 0, 0};

  // k = A/L * E, with A the cross sectional area (=4R²) and L the free length
  // between particles (=2*R).
  const double stiffness = 2.0 * FLAGS_sphere_radius * FLAGS_elastic_modulus;

  // Estimate dissipation form desired damping ratio.
  const double critical_dissipation =
      2.0 * std::sqrt(stiffness * mass);
  const double linear_dissipation =
      FLAGS_linear_damping_ratio * critical_dissipation;

  const double w0 = std::sqrt(stiffness / mass);
  const double T0 = 2.0 * M_PI / w0;
  PRINT_VAR(T0);

  for (int i = 0; i < FLAGS_grid_i; ++i) {
    for (int j = 0; j < FLAGS_grid_j; ++j) {
      for (int k = 0; k < FLAGS_grid_k; ++k) {
        const std::tuple<int, int, int> index = std::make_tuple(i, j, k);

        const Body<double>& body = plant->get_body(bodies.at(index));

        for (int di = -1; di <= 1; ++di) {
          for (int dj = -1; dj <= 1; ++dj) {
            for (int dk = -1; dk <= 1; ++dk) {
              if ((di == 0) && (dj == 0) && (dk == 0)) continue;  // skip this sphere.

              // If the neighbor lies outside the grid, we are at a boundary.
              // Therefore skip adding force element.
              if (i + di < 0 || i + di >= FLAGS_grid_i || j + dj < 0 ||
                  j + dj >= FLAGS_grid_j || k + dk < 0 ||
                  k + dk >= FLAGS_grid_k)
                continue;                              

              const std::tuple<int, int, int> neighbor_index =
                  std::make_tuple(i + di, j + dj, k + dk);

              const Body<double>& neighbor =
                  plant->get_body(bodies.at(neighbor_index));

              // What if we put the points on the surface?
              double rest_length =
                  std::sqrt(std::abs(di) + std::abs(dj) + std::abs(dk)) * 2 *
                  FLAGS_sphere_radius;

              plant->AddForceElement<LinearSpringDamper>(
                  body, Vector3d::Zero(), neighbor, Vector3d::Zero(), rest_length, stiffness, linear_dissipation, FLAGS_cutoff_length);
            }
          }
        }
      }
    }
  }
  return bodies;
}

void set_grid_poses(
    const std::map<std::tuple<int, int, int>, multibody::BodyIndex>& bodies,
    const Vector3<double> grid_origin, multibody::MultibodyPlant<double>* plant,
    systems::Context<double>* plant_context) {
  Vector3d corner(-FLAGS_sphere_radius * FLAGS_grid_i,
                  -FLAGS_sphere_radius * FLAGS_grid_j,
                  0);
  const Vector3d offset(FLAGS_sphere_radius, FLAGS_sphere_radius,
                        FLAGS_sphere_radius);

  for (int i = 0; i < FLAGS_grid_i; ++i) {
    for (int j = 0; j < FLAGS_grid_j; ++j) {
      for (int k = 0; k < FLAGS_grid_k; ++k) {
        const std::tuple<int, int, int> index = std::make_tuple(i, j, k);

        const Body<double>& body = plant->get_body(bodies.at(index));

        Vector3d origin =
            grid_origin + corner + offset +
            Vector3d(2 * i * FLAGS_sphere_radius, 2 * j * FLAGS_sphere_radius,
                     2 * k * FLAGS_sphere_radius);

        const auto& joint =
            plant->GetJointByName<ParticleJoint>("joint_" + body.name());
        joint.set_position(plant_context, origin);

#if 0
        plant->SetFreeBodyPose(
            plant_context, &plant_state, body,
            math::RigidTransform<double>(
                math::RollPitchYaw<double>(0, 0, -1.57), origin));
#endif                
      }
    }
  }
}

void DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph =
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Load and parse double pendulum SDF from file into a tree.
  multibody::MultibodyPlant<double>* dp =
      builder.AddSystem<multibody::MultibodyPlant<double>>(FLAGS_mbp_dt);
  dp->set_name("plant");
  dp->RegisterAsSourceForSceneGraph(&scene_graph);

  multibody::Parser parser(dp);

  // Add knife model ------------------------------------------------------
  const std::string sdf_path = FindResourceOrThrow(kDoublePendulumSdfPath);
  const auto model_instance_index = parser.AddModelFromFile(sdf_path);
  (void)model_instance_index;

  const auto& root_link = dp->GetBodyByName("base");
  dp->AddJoint<multibody::WeldJoint>(
      "weld_base", dp->world_body(), std::nullopt, root_link, std::nullopt,
      math::RigidTransform<double>(math::RollPitchYaw<double>(0, 0, -1.57),
                                   Eigen::Vector3d(0.05, 0.0, 0.75)));
  dp->AddJointActuator("a1", dp->GetJointByName("knife_joint"));

  // Add knife model ------------------------------------------------------

  // Add sphere grid
  const double sphere_volume = 4.0 / 3.0 * M_PI * FLAGS_sphere_radius *
                               FLAGS_sphere_radius * FLAGS_sphere_radius;
  const double particle_mass = FLAGS_particle_density * sphere_volume;
  PRINT_VAR(particle_mass);
  std::map<std::tuple<int, int, int>, multibody::BodyIndex> bodies = make_grid(
      particle_mass, Vector4<double>(0.53, 0.01, 0.53, 1.0), dp, &scene_graph);

  if (FLAGS_simple_table) {
    const RigidTransform<double> X_WT{Vector3<double>{0, 0, 0.75}};
    dp->RegisterCollisionGeometry(dp->world_body(), X_WT, geometry::HalfSpace(),
                                  "ground",
                                  multibody::CoulombFriction<double>{});
    dp->RegisterVisualGeometry(dp->world_body(), X_WT, geometry::HalfSpace(),
                               "ground", geometry::IllustrationProperties());
  } else {
    const double dx_table_center_to_robot_base = 0.3257;
    const double dz_table_top_robot_base = 0.0127;
    const std::string table_sdf_path = FindResourceOrThrow(
        "drake/examples/cutting_sim/models/"
        "extra_heavy_duty_table_surface_only_collision.sdf");

    const auto table_instance_index = parser.AddModelFromFile(table_sdf_path);
    const auto& child_frame = dp->GetFrameByName("link", table_instance_index);

    RigidTransform<double> X_WT(Eigen::Vector3d(dx_table_center_to_robot_base,
                                                0, -dz_table_top_robot_base));

    dp->WeldFrames(dp->world_frame(), child_frame, X_WT);
  }
  //dp->set_penetration_allowance(0.002);

  dp->Finalize();

  PRINT_VAR(dp->num_positions());
  PRINT_VAR(dp->num_velocities());

  // PID for knife-----------------------------------------------
  const Eigen::VectorXd Kp = Eigen::VectorXd::Ones(1) * FLAGS_Kp_;
  const Eigen::VectorXd Ki = Eigen::VectorXd::Ones(1) * FLAGS_Ki_;
  const Eigen::VectorXd Kd = Eigen::VectorXd::Ones(1) * FLAGS_Kd_;
  const auto* const pid =
      builder.AddSystem<systems::controllers::PidController<double>>(
          Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Identity(1, 1), Kp,
          Ki, Kd);

  builder.Connect(dp->get_state_output_port(model_instance_index),
                  pid->get_input_port_estimated_state());
  builder.Connect(pid->get_output_port_control(),
                  dp->get_actuation_input_port(model_instance_index));

  auto desired_base_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(
          Eigen::VectorXd::Ones(2) * 0.00f);
  builder.Connect(desired_base_source->get_output_port(),
                  pid->get_input_port_desired_state());
  // PID for knife---------------------------------------------------

  DRAKE_DEMAND(!!dp->get_source_id());
  builder.Connect(
      dp->get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(dp->get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  dp->get_geometry_query_input_port());

  const geometry::DrakeVisualizerParams params{
      .publish_period = 1 / FLAGS_vis_rate,
      .role = geometry::Role::kIllustration,
      .default_color = geometry::Rgba(0.9, 0.9, 0.9, 1.0)};
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, nullptr,
                                           params);
  // Add contact forces viz.
  if (FLAGS_forces_viz) {
    ConnectContactResultsToDrakeVisualizer(&builder, *dp);
  }

  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(*dp, diagram_context.get());

  //systems::State<double>& plant_state = diagram->GetMutableSubsystemState(
    //  *dp, &diagram_context->get_mutable_state());

  // We define the grid origin as lying in the center of the bottom face.
  const Vector3d grid_origin(FLAGS_grid_x, FLAGS_grid_y, FLAGS_grid_z);

  // set spheres poses
  set_grid_poses(bodies, grid_origin, dp, &plant_context);

  // Set init position
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(1);
  positions[0] = 1.57;
  dp->SetPositions(&plant_context, model_instance_index, positions);

  std::unique_ptr<Simulator<double>> simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));
  simulator->AdvanceTo(FLAGS_simulation_time);

  // Report some stats to stdout.
  PrintSimulatorStatistics(*simulator);
}
}  // namespace
}  // namespace cutting_potatoe
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  gflags::SetUsageMessage(
      "Simple sdformat usage example, just"
      "make sure drake-visualizer is running");

  FLAGS_simulator_accuracy = 0.1;
  FLAGS_simulator_max_time_step = 0.1;
  FLAGS_simulator_integration_scheme = "runge_kutta3";
  FLAGS_simulator_publish_every_time_step = true;

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::cutting_potatoe::DoMain();
  return 0;
}
