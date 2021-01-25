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
#include "drake/multibody/tree/cut_magnet.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(target_realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 30, "How long to simulate the pendulum");
DEFINE_double(max_time_step, 0.001, "Simulation time step used for integrator.");

DEFINE_double(Kp_, 15.0, "Kp");
DEFINE_double(Ki_, 0.0, "Ki");
DEFINE_double(Kd_, 0.0, "Kd");

DEFINE_double(force_scale_, 20.0, "force scale for magnet");
DEFINE_double(turn_off_force_threshold_, 0.001,
              "Distance threshold to turn off magnets");
DEFINE_int32(grid_i, 4, "Number of spheres along x dimension of grid.");
DEFINE_int32(grid_j, 4, "Number of spheres along y dimension of grid.");
DEFINE_int32(grid_k, 2, "Number of spheres along z dimension of grid.");
DEFINE_double(sphere_radius, 0.0125, "Sphere radius.");
DEFINE_double(sphere_mass, 0.6, "Sphere mass.");

namespace drake {
namespace examples {
namespace cutting_potatoe {
namespace {

using drake::multibody::Body;
using drake::multibody::CutMagnet;
using drake::multibody::SpatialInertia;
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

static const char* const kDoublePendulumSdfPath =
    "drake/examples/cutting_sim/models/guillotine.sdf";

std::map<std::tuple<int, int, int>, multibody::BodyIndex> make_grid(
    double mass, const Vector4<double>& color,
    multibody::MultibodyPlant<double>* plant) {
  const multibody::CoulombFriction<double> sphere_friction(0.5, 0.3);

  const double sphere_ixyz = .4 * FLAGS_sphere_radius * FLAGS_sphere_radius;

  std::map<std::tuple<int, int, int>, multibody::BodyIndex> bodies;

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

        plant->RegisterCollisionGeometry(sphere, RigidTransform<double>(),
                                         geometry::Sphere(FLAGS_sphere_radius),
                                         fmt::format("{}_collision", name),
                                         sphere_friction);

        plant->RegisterVisualGeometry(sphere, RigidTransform<double>(),
                                      geometry::Sphere(FLAGS_sphere_radius),
                                      fmt::format("{}_visual", name), color);
        bodies[index] = sphere.index();
      }
    }
  }

  // Add force elements between neighboring spheres.
  const Vector3<double> p_AP_{0, 0, 0};
  const Vector3<double> p_BQ_{0, 0, 0};

  for (int i = 0; i < FLAGS_grid_i; ++i) {
    for (int j = 0; j < FLAGS_grid_j; ++j) {
      for (int k = 0; k < FLAGS_grid_k; ++k) {
        const std::tuple<int, int, int> index = std::make_tuple(i, j, k);

        const Body<double>& body = plant->get_body(bodies.at(index));

        for (int di = -1; di <= 1; ++di) {
          for (int dj = -1; dj <= 1; ++dj) {
            for (int dk = -1; dk <= 1; ++dk) {
              if ((di == 0) && (dj == 0) && (dk == 0)) continue;  // skip this sphere.

              if (i + di < 0 || i + di >= FLAGS_grid_i || j + dj < 0 ||
                  j + dj >= FLAGS_grid_j || k + dk < 0 ||
                  k + dk >= FLAGS_grid_k)
                continue;

              const std::tuple<int, int, int> neighbor_index =
                  std::make_tuple(i + di, j + dj, k + dk);

              const Body<double>& neighbor =
                  plant->get_body(bodies.at(neighbor_index));

              double rest_length =
                  std::sqrt(std::abs(di) + std::abs(dj) + std::abs(dk)) * 2 *
                  FLAGS_sphere_radius;

              plant->AddForceElement<drake::multibody::CutMagnet>(
                  body, p_AP_, neighbor, p_BQ_, FLAGS_force_scale_, 1.0,
                  rest_length + FLAGS_turn_off_force_threshold_);
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
    systems::Context<double>& plant_context,
    systems::State<double>& plant_state) {
  Vector3d corner(-FLAGS_sphere_radius * FLAGS_grid_i,
                  -FLAGS_sphere_radius * FLAGS_grid_j,
                  -FLAGS_sphere_radius * FLAGS_grid_k);
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

        plant->SetFreeBodyPose(
            plant_context, &plant_state, body,
            math::RigidTransform<double>(
                math::RollPitchYaw<double>(0, 0, -1.57), origin));
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
      builder.AddSystem<multibody::MultibodyPlant<double>>(FLAGS_max_time_step);
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
  std::map<std::tuple<int, int, int>, multibody::BodyIndex> bodies =
      make_grid(FLAGS_sphere_mass, Vector4<double>(0.53, 0.01, 0.53, 1.0), dp);

  const double dx_table_center_to_robot_base = 0.3257;
  const double dz_table_top_robot_base = 0.0127;
  const std::string table_sdf_path = FindResourceOrThrow(
      "drake/examples/cutting_sim/models/"
      "extra_heavy_duty_table_surface_only_collision.sdf");

  const auto table_instance_index = parser.AddModelFromFile(table_sdf_path);
  const auto& child_frame = dp->GetFrameByName("link", table_instance_index);

  RigidTransform<double> X_WT(Eigen::Vector3d(dx_table_center_to_robot_base, 0,
                                              -dz_table_top_robot_base));

  dp->WeldFrames(dp->world_frame(), child_frame, X_WT);

  dp->set_penetration_allowance(0.002);

  dp->Finalize();

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

  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(*dp, diagram_context.get());

  systems::State<double>& plant_state = diagram->GetMutableSubsystemState(
      *dp, &diagram_context->get_mutable_state());

  const Vector3d grid_origin(0.26, 0.0, 0.81);

  // set spheres poses
  set_grid_poses(bodies, grid_origin, dp, plant_context, plant_state);

  // Set init position
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(1);
  positions[0] = 1.57;
  dp->SetPositions(&plant_context, model_instance_index, positions);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);
}
}  // namespace
}  // namespace cutting_potatoe
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  gflags::SetUsageMessage(
      "Simple sdformat usage example, just"
      "make sure drake-visualizer is running");

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::cutting_potatoe::DoMain();
  return 0;
}
