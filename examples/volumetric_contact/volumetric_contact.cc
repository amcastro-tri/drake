#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "fmt/ostream.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/sine.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

namespace drake {
namespace examples {
namespace simple_gripper {
namespace {

using Eigen::Vector3d;
using geometry::SceneGraph;
using geometry::Sphere;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::AddMultibodyPlantSceneGraph;
using multibody::Body;
using multibody::CoulombFriction;
using multibody::ConnectContactResultsToDrakeVisualizer;
using multibody::MultibodyPlant;
using multibody::UnitInertia;
using multibody::RigidBody;
using multibody::SpatialForce;
using multibody::SpatialInertia;
using multibody::PrismaticJoint;
using systems::ImplicitEulerIntegrator;
using systems::RungeKutta2Integrator;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;
using systems::Sine;

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation. [s].");
DEFINE_double(penetration, 0.001,
              "The maximum penetration of the object against the plane. [m].");

// Contact parameters
DEFINE_double(v_stiction_tolerance, 1.0e-4,
              "The maximum slipping speed allowed during stiction. [m/s]");
DEFINE_double(friction_coefficient, 1.0, "The coefficient of friction.");
DEFINE_double(elastic_modulus, 2.5e5, "Ground's elastic modulus, in Pa.");
DEFINE_double(dissipation, 5.0, "Ground's dissipation, in s/m.");
DEFINE_double(ground_size, 0.5,
              "The size of the box used to model the ground.");


// Rigid object type and parameters.
// Only relevant parameters are used for a given object type.
DEFINE_string(rigid_object_shape, "sphere",
              "Object shape. Available options are: "
              "'sphere','vertical_cylinder','parallel_cylinder'");
DEFINE_double(radius, 0.05, "Radius.");  // sphere, cylinder.
DEFINE_double(length, 0.5, "Length.");  // cylinder. L >> R.

void AddSoftGround(double size, double friction_coefficient,
                   MultibodyPlant<double>* plant) {
  const Vector3d p_WG(0.0, 0.0, -size / 2.0);
  const RigidTransformd X_WG(p_WG);
  const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
  plant->RegisterVisualGeometry(plant->world_body(), X_WG,
                                geometry::Box(size, size, size),
                                "SoftGroundVisualGeometry", green);
  geometry::GeometryId ground_id = plant->RegisterCollisionGeometry(
      plant->world_body(), X_WG, geometry::Box(size, size, size),
      "SoftGroundCollisionGeometry",
      CoulombFriction<double>(friction_coefficient, friction_coefficient));

  plant->set_elastic_modulus(ground_id, FLAGS_elastic_modulus);
  plant->set_hydroelastics_dissipation(ground_id, FLAGS_dissipation);    
}

const RigidBody<double>& AddRigidObject(MultibodyPlant<double>* plant) {
  auto make_shape = []() -> std::unique_ptr<geometry::Shape> {
    if (FLAGS_rigid_object_shape == "sphere") {
      return std::make_unique<geometry::Sphere>(FLAGS_radius);
    } else if (FLAGS_rigid_object_shape == "vertical_cylinder") {
      return std::make_unique<geometry::Cylinder>(FLAGS_radius, FLAGS_length);
    } else {
      DRAKE_UNREACHABLE();
    }
  };

  // Inertial properties are arbitrary since hydro forces are only state
  // dependent.
  const double mass = 1.0;
  const double length = 1.0;
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBcm = UnitInertia<double>::SolidSphere(length);
  const SpatialInertia<double> M_BBcm_B(mass, p_BoBcm_B, G_BBcm);

  // Create a rigid body B with the mass properties of a uniform sphere.
  const RigidBody<double>& body = plant->AddRigidBody("body", M_BBcm_B);

  // Body B's visual geometry and collision geometry are a sphere.
  // The pose X_BG of block B's geometry frame G is an identity transform.
  auto shape = make_shape();
  const RigidTransformd X_BG;  // Identity transform.
  const Vector4<double> lightBlue(0.5, 0.8, 1.0, 1.0);
  plant->RegisterVisualGeometry(body, X_BG, *shape, "RigidBodyVisualGeometry",
                                lightBlue);
  plant->RegisterCollisionGeometry(
      body, X_BG, *shape, "RigidBodyCollisionGeometry",
      CoulombFriction<double>(FLAGS_friction_coefficient,
                              FLAGS_friction_coefficient));
  return body;
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto items = AddMultibodyPlantSceneGraph(&builder);
  MultibodyPlant<double>& plant = items.plant;
  SceneGraph<double>& scene_graph = items.scene_graph;
  scene_graph.set_name("scene_graph");

  AddSoftGround(FLAGS_ground_size, FLAGS_friction_coefficient, &plant);
  const auto& body = AddRigidObject(&plant);
  plant.use_hydroelastic_model(true);
  plant.set_stiction_tolerance(FLAGS_v_stiction_tolerance);
  plant.Finalize();  

#if 0
  // Print maximum time step and the time scale introduced by the compliance in
  // the contact model as a reference to the user.
  fmt::print("Maximum time step = {:10.6f} s\n", max_time_step);
  fmt::print("Compliance time scale = {:10.6f} s\n",
             plant.get_contact_penalty_method_time_scale());
#endif

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  DrakeLcm lcm;
  geometry::ConnectDrakeVisualizer(&builder, scene_graph, &lcm);

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());  

  const auto& draw_publisher = diagram->GetSubsystemByName("draw_publisher");
  const systems::Context<double>& publisher_context =
      diagram->GetMutableSubsystemContext(draw_publisher, diagram_context.get());  

  // For vertical cylinder. Modify for other shapes.
  RigidTransformd X_WB(Vector3d(0.0, 0.0, FLAGS_length / 2.0 - FLAGS_penetration));
  plant.SetFreeBodyPose(&plant_context, body, X_WB);

  // PRINT_VAR();

  geometry::DispatchLoadMessage(scene_graph, &lcm);
  draw_publisher.Publish(publisher_context);

  std::vector<SpatialForce<double>> F_BBo_W_array(plant.num_bodies(),
                                                  SpatialForce<double>::Zero());
  plant.CalcAndAddHydroelasticsContactForces(plant_context, &F_BBo_W_array);

  const SpatialForce<double>& F_BBo_W = F_BBo_W_array[body.node_index()];
  PRINT_VAR(F_BBo_W);

  return 0;
}

}  // namespace
}  // namespace simple_gripper
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("Setup to compute volumetric forces.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::simple_gripper::do_main();
}
