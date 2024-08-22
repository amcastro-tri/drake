#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/multibody/deformable/parallel_gripper_controller.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/camera_config.h"
#include "drake/systems/sensors/camera_config_functions.h"

DEFINE_double(simulation_time, 4.0, "Desired duration of the simulation [s].");
DEFINE_double(realtime_rate, 0.0, "Desired real time rate.");
DEFINE_double(discrete_time_step, 2.5e-2,
              "Discrete time step for the system [s].");
DEFINE_bool(render_bubble, true,
            "Renders the dot pattern inside the bubble gripper if true.");

DEFINE_bool(visualize, true, "Publishes for visualization.");
DEFINE_bool(log_data, true, "Logs forces and teddy state.");

using drake::examples::deformable::ParallelGripperController;
using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::GeometryInstance;
using drake::geometry::IllustrationProperties;
using drake::geometry::Mesh;
using drake::geometry::PerceptionProperties;
using drake::geometry::ProximityProperties;
using drake::geometry::RenderEngineGlParams;
using drake::geometry::Rgba;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::Body;
using drake::multibody::ContactResults;
using drake::multibody::CoulombFriction;
using drake::multibody::DeformableBodyId;
using drake::multibody::DeformableModel;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::PackageMap;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::fem::DeformableBodyConfig;
using drake::schema::Transform;
using drake::systems::Context;
using drake::systems::sensors::ApplyCameraConfig;
using drake::systems::sensors::CameraConfig;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace drake {
namespace examples {
namespace multibody {
namespace bubble_gripper {
namespace {

int do_main() {
  if (!drake::geometry::kHasRenderEngineGl && FLAGS_render_bubble) {
    drake::log()->error(
        "This example can only be run on Linux when the bubble gripper is "
        "rendered.");
    return 0;
  }
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  DRAKE_DEMAND(FLAGS_discrete_time_step > 0.0);
  plant_config.time_step = FLAGS_discrete_time_step;
  /* Deformable simulation only works with SAP solver. */
  plant_config.discrete_contact_approximation = "sap";

  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  if (FLAGS_render_bubble) {
    /* Add a renderer to render the inside dot pattern of the bubble gripper.
     Currently (April 2024), deformable rendering is only supported by
     RenderEngineGl. */
    scene_graph.AddRenderer(
        "gl_renderer", geometry::MakeRenderEngineGl(RenderEngineGlParams{}));
  }

  /* Minimum required proximity properties for rigid bodies to interact with
   deformable bodies.
   1. A valid Coulomb friction coefficient, and
   2. A resolution hint. (Rigid bodies need to be tessellated so that collision
   queries can be performed against deformable geometries.) */
  ProximityProperties rigid_proximity_props;
  const CoulombFriction<double> surface_friction(1.0, 1.0);
  const double resolution_hint = 0.01;
  AddContactMaterial({}, {}, surface_friction, &rigid_proximity_props);
  rigid_proximity_props.AddProperty(geometry::internal::kHydroGroup,
                                    geometry::internal::kRezHint,
                                    resolution_hint);

  /* Set up a ground. */
  Box ground{1, 1, 1};
  const RigidTransformd X_WG(Eigen::Vector3d{0, 0, -0.505});
  plant.RegisterCollisionGeometry(plant.world_body(), X_WG, ground,
                                  "ground_collision", rigid_proximity_props);
  IllustrationProperties illustration_props;
  illustration_props.AddProperty("phong", "diffuse",
                                 Vector4d(0.95, 0.80, 0.65, 0.9));
  /* Avoid rendering the ground as it clutters the background.
   Currently, all visual geometries added through MultibodyPlant are
   automatically assigned perception properties. When that automatic assignment
   is no longer done, we can remove this and simply not assign a perception
   property. */
  illustration_props.AddProperty("renderer", "accepting",
                                 std::set<std::string>{"nothing"});
  plant.RegisterVisualGeometry(plant.world_body(), X_WG, ground,
                               "ground_visual", illustration_props);

  /* Parse the gripper model (without the bubbles). */
  Parser parser(&plant, &scene_graph);
  ModelInstanceIndex gripper_instance = parser.AddModelsFromUrl(
      "package://drake_models/wsg_50_description/sdf/"
      "schunk_wsg_50_deformable_bubble.sdf")[0];

  /* Add in the bubbles. */
  DeformableBodyConfig<double> bubble_config;
  bubble_config.set_youngs_modulus(1e4);                  // [Pa]
  bubble_config.set_poissons_ratio(0.45);                 // unitless
  bubble_config.set_mass_density(10);                     // [kg/m³]
  bubble_config.set_stiffness_damping_coefficient(0.05);  // [1/s]

  /* Minimally required proximity properties for deformable bodies: A valid
   Coulomb friction coefficient. */
  ProximityProperties deformable_proximity_props;
  AddContactMaterial({}, {}, surface_friction, &deformable_proximity_props);

  /* The mesh we render in the camera sim. */
  PerceptionProperties perception_properties;
  const std::string textured_bubble_obj = PackageMap{}.ResolveUrl(
      "package://drake_models/wsg_50_description/meshes/textured_bubble.obj");
  /* Assign the mesh to be rendered. If this property is not specified, the
   untextured surface mesh of the simulated volume mesh is rendered. */
  perception_properties.AddProperty("deformable", "embedded_mesh",
                                    textured_bubble_obj);

  /* Add in the left bubble. */
  const std::string bubble_vtk = PackageMap{}.ResolveUrl(
      "package://drake_models/wsg_50_description/meshes/bubble.vtk");
  auto left_bubble_mesh = std::make_unique<Mesh>(bubble_vtk);
  /* Pose of the left bubble (at initialization) in the world frame. */
  const RigidTransformd X_WBl(RollPitchYawd(M_PI_2, M_PI, 0),
                              Vector3d(-0.185, -0.09, 0.06));
  auto left_bubble_instance = std::make_unique<GeometryInstance>(
      X_WBl, std::move(left_bubble_mesh), "left bubble");
  left_bubble_instance->set_proximity_properties(deformable_proximity_props);
  left_bubble_instance->set_perception_properties(perception_properties);
  /* Since the deformable geometry is specified through Shape::Mesh, the
   resolution hint is unused. */
  const double unused_resolution_hint = 1.0;
  DeformableModel<double>& deformable_model = plant.mutable_deformable_model();
  const DeformableBodyId left_bubble_id =
      deformable_model.RegisterDeformableBody(std::move(left_bubble_instance),
                                              bubble_config,
                                              unused_resolution_hint);

  /* Now we attach the bubble to the WSG finger using a fixed constraint. To do
   that, we specify a box geometry and put all vertices of the bubble geometry
   under fixed constraint with the rigid finger if they fall inside the box.
   Refer to DeformableModel::AddFixedConstraint for details. */
  const Body<double>& left_finger = plant.GetBodyByName("left_finger");
  /* Pose of the bubble in the left finger body frame. */
  const RigidTransformd X_FlBl = RigidTransformd(
      math::RollPitchYawd(M_PI_2, M_PI_2, 0), Vector3d(0.0, -0.03, -0.1125));
  /* All vertices of the deformable bubble mesh inside this box will be subject
   to fixed constraints. */
  const Box box(0.1, 0.004, 0.15);
  deformable_model.AddFixedConstraint(
      left_bubble_id, left_finger, X_FlBl, box,
      /* The pose of the box in the left finger's frame. */
      RigidTransformd(Vector3d(0.0, -0.03, -0.1)));

  /* Add in the right bubble and attach it to the right finger. */
  auto right_bubble_mesh = std::make_unique<Mesh>(bubble_vtk);
  /* Pose of the right bubble (at initialization) in the world frame. */
  const RigidTransformd X_WBr(RollPitchYawd(-M_PI_2, M_PI, 0),
                              Vector3d(-0.185, 0.09, 0.06));
  auto right_bubble_instance = std::make_unique<GeometryInstance>(
      X_WBr, std::move(right_bubble_mesh), "right bubble");
  right_bubble_instance->set_proximity_properties(deformable_proximity_props);
  right_bubble_instance->set_perception_properties(perception_properties);
  const DeformableBodyId right_bubble_id =
      deformable_model.RegisterDeformableBody(std::move(right_bubble_instance),
                                              bubble_config,
                                              unused_resolution_hint);
  const Body<double>& right_finger = plant.GetBodyByName("right_finger");
  /* Pose of the right finger body (at initialization) in the world frame. */
  const RigidTransformd X_FrBr = RigidTransformd(
      math::RollPitchYawd(-M_PI_2, M_PI_2, 0), Vector3d(0.0, 0.03, -0.1125));
  deformable_model.AddFixedConstraint(
      right_bubble_id, right_finger, X_FrBr, box,
      /* The pose of the box in the right finger's frame. */
      RigidTransformd(Vector3d(0.0, 0.03, -0.1)));

  /* Add in a deformable manipuland. */
  DeformableBodyConfig<double> teddy_config;
  teddy_config.set_youngs_modulus(5e4);                  // [Pa]
  teddy_config.set_poissons_ratio(0.45);                 // unitless
  teddy_config.set_mass_density(1000);                   // [kg/m³]
  teddy_config.set_stiffness_damping_coefficient(0.05);  // [1/s]
  const std::string teddy_vtk = FindResourceOrThrow(
      "drake/examples/multibody/deformable/models/teddy.vtk");
  auto teddy_mesh = std::make_unique<Mesh>(teddy_vtk, /* scale */ 0.15);
  auto teddy_instance = std::make_unique<GeometryInstance>(
      RigidTransformd(math::RollPitchYawd(M_PI / 2.0, 0, -M_PI / 2.0),
                      Vector3d(-0.17, 0, 0)),
      std::move(teddy_mesh), "teddy");
  teddy_instance->set_proximity_properties(deformable_proximity_props);
  /* Give the teddy bear a brown color for illustration to better distinguish it
   from the bubble gripper in the visualizer. */
  IllustrationProperties teddy_illustration_props;
  teddy_illustration_props.AddProperty("phong", "diffuse",
                                       Rgba(0.82, 0.71, 0.55, 1.0));
  teddy_instance->set_illustration_properties(teddy_illustration_props);
  const DeformableBodyId teddy_id = deformable_model.RegisterDeformableBody(
      std::move(teddy_instance), teddy_config, 1.0);

  /* All rigid and deformable models have been added. Finalize the plant. */
  plant.Finalize();

  /* Set the width between the fingers for open and closed states as well as the
   height to which the gripper lifts the manipuland. */
  const double open_width = 0.12;
  const double closed_width = 0.04;
  const double lifted_height = 0.12;
  const auto& control = *builder.AddSystem<ParallelGripperController>(
      open_width, closed_width, lifted_height);
  builder.Connect(control.get_output_port(),
                  plant.get_desired_state_input_port(gripper_instance));

  /* Add a visualizer that emits LCM messages for visualization. */
  if (FLAGS_visualize) {
    geometry::DrakeVisualizerParams params;
    params.publish_period = std::min(1.0 / 30.0, FLAGS_discrete_time_step);
    params.role = geometry::Role::kIllustration;
    geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, nullptr,
                                             params);
  }

  /* We want to look in the -Py direction so we line up Bz with -Py.*/
  const Vector3d Bz_P = -Vector3d::UnitY();
  const Vector3d Bx_P = Vector3d::UnitZ();
  const Vector3d By_P = Bz_P.cross(Bx_P);  // Already a unit vector.
  const Vector3d p_PB(0, 0.06, -0.11);
  const RotationMatrixd R_PB =
      RotationMatrixd::MakeFromOrthonormalColumns(Bx_P, By_P, Bz_P);
  Transform schema_X_PB(RigidTransformd(R_PB, p_PB));
  schema_X_PB.base_frame = "right_finger";

  /* Create the camera if rendering is requested. */
  if (FLAGS_render_bubble) {
    const CameraConfig camera_config{.width = 1280,
                                     .height = 720,
                                     .focal{CameraConfig::FovDegrees{.y = 90}},
                                     .clipping_near = 0.001,
                                     .X_PB = schema_X_PB,
                                     .renderer_name = "gl_renderer",
                                     .show_rgb = true};
    ApplyCameraConfig(camera_config, &builder);
  }

  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  /* Build the simulator and run! */
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  /* Set the initial conditions for the spatula pose and the gripper finger
   positions. */
  Context<double>& mutable_root_context = simulator.get_mutable_context();
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, &mutable_root_context);

  /* Set initial finger joint positions to the "open" state. */
  const PrismaticJoint<double>& right_joint =
      plant.GetMutableJointByName<PrismaticJoint>("right_finger_sliding_joint");
  const PrismaticJoint<double>& left_joint =
      plant.GetMutableJointByName<PrismaticJoint>("left_finger_sliding_joint");
  left_joint.set_translation(&plant_context, -open_width / 2.0);
  right_joint.set_translation(&plant_context, open_width / 2.0);

  /* Define a monitor to log state and forces. */
  std::fstream forces_file{"forces.dat", forces_file.out};
  std::fstream teddy_file{"teddy_state.dat", forces_file.out};
  if (FLAGS_log_data) {
    const MultibodyPlant<double>* plant_ptr = &plant;
    const geometry::GeometryId right_bubble_geometry_id =
        deformable_model.GetGeometryId(right_bubble_id);
    const geometry::GeometryId left_bubble_geometry_id =
        deformable_model.GetGeometryId(left_bubble_id);
    simulator.set_monitor([&](const systems::Context<double>& root_context) {
      const systems::DiscreteStateIndex teddy_state_index =
          deformable_model.GetDiscreteStateIndex(teddy_id);
      const geometry::GeometryId teddy_geometry_id =
          deformable_model.GetGeometryId(teddy_id);

      (void)teddy_state_index;

      const auto& contact_results =
          plant_ptr->get_contact_results_output_port()
              .Eval<ContactResults<double>>(plant_context);

      // Accumulate the total contact force on the teddy bear.
      Vector3d f_Teddy_W = Vector3d::Zero();
      Vector3d f_left = Vector3d::Zero();
      Vector3d f_right = Vector3d::Zero();
      int num_contacts = 0;
      for (int i = 0; i < contact_results.num_deformable_contacts(); ++i) {
        const auto& info = contact_results.deformable_contact_info(i);
        const geometry::GeometryId id_A = info.id_A();
        const geometry::GeometryId id_B = info.id_B();

        num_contacts += info.contact_mesh().num_elements();

        if (id_A == teddy_geometry_id) {
          f_Teddy_W += info.F_Ac_W().translational();
        }

        if (id_B == teddy_geometry_id) {
          f_Teddy_W -= info.F_Ac_W().translational();
        }

        if (id_A == left_bubble_geometry_id) {
          f_left = info.F_Ac_W().translational();
        }
        if (id_B == left_bubble_geometry_id) {
          f_left = -info.F_Ac_W().translational();
        }

        if (id_A == right_bubble_geometry_id) {
          f_right = info.F_Ac_W().translational();
        }
        if (id_B == right_bubble_geometry_id) {
          f_right = -info.F_Ac_W().translational();
        }
      }

      forces_file << fmt::format(
          "{} {} {} {} {}\n", root_context.get_time(),
          fmt_eigen(f_left.transpose()), fmt_eigen(f_right.transpose()),
          fmt_eigen(f_Teddy_W.transpose()), num_contacts);

      // Log the teddy bear state.
      const VectorX<double>& teddy_state =
          plant_context.get_discrete_state(teddy_state_index).value();
      teddy_file << fmt::format("{} {}\n", root_context.get_time(),
                                fmt_eigen(teddy_state.transpose()));

      return systems::EventStatus::Succeeded();
    });
  }

  using clock = std::chrono::steady_clock;

  // Compute something to trigger reification etc so that we don't measure it as
  // part of AdvanceTo().
  clock::time_point start = clock::now();
  const auto& contact_results =
      plant.get_contact_results_output_port().Eval<ContactResults<double>>(
          plant_context);
  clock::time_point end = clock::now();
  fmt::print("Num deformable contacts at t = 0: {}. {} secs.\n",
             contact_results.num_deformable_contacts(),
             std::chrono::duration<double>(end - start).count());

  simulator.Initialize();
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);

  start = clock::now();
  simulator.AdvanceTo(FLAGS_simulation_time);
  end = clock::now();
  const double wall_clock_time =
      std::chrono::duration<double>(end - start).count();
  fmt::print("Simulator::AdvanceTo() wall clock time: {} seconds.\n",
             wall_clock_time);

  forces_file.close();
  teddy_file.close();

  return 0;
}

}  // namespace
}  // namespace bubble_gripper
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "This is a demo used to showcase the following features in deformable "
      "body simulation in Drake:\n"
      "  1. frictional contact resolution among deformable bodies;\n"
      "  2. deformable geometry rendering;\n"
      "  3. fixed constraints between rigid bodies and deformable bodies.\n"
      "Note that this example only runs on Linux systems.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::bubble_gripper::do_main();
}
