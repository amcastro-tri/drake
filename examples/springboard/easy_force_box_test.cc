#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/examples/springboard/easy_force.h"

namespace drake {
namespace examples {
namespace {

using geometry::SceneGraph;
using lcm::DrakeLcm;
using Eigen::Vector3d;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::MultibodyPlant;
using drake::multibody::UniformGravityFieldElement;

DEFINE_double(target_realtime_rate, 1.0,
            "Desired rate relative to real time.  See documentation for "
            "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(duration, 5.0,
            "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

DEFINE_double(box_l, 1, "Side length of Box");
DEFINE_double(box_m, 1, "Mass of Box");

DEFINE_bool(rand_rpy, false, "Randomize initial Box rotation");

DEFINE_double(Tx, 0, "Torque in x-direction");
DEFINE_double(Ty, 0, "Torque in y-direction");
DEFINE_double(Tz, 0, "Torque in z-direction");

DEFINE_double(Fx, 10, "Force in x-direction");
DEFINE_double(Fy, 10, "Force in y-direction");
DEFINE_double(Fz, 10, "Force in z-direction");

int do_main() {
    systems::DiagramBuilder<double> builder;

    auto pair = AddMultibodyPlantSceneGraph(&builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));
    MultibodyPlant<double>& plant = pair.plant;

    SceneGraph<double>& scene_graph = pair.scene_graph;
    scene_graph.set_name("scene_graph");

    // Add gravity to the model.
    // Deprecated: please see deprecation message when building.
    //plant.AddForceElement<UniformGravityFieldElement>();

    // I set gravity to zero so that it's easier for me to see if the external
    // force is working (or not!).
    plant.mutable_gravity_field().set_gravity_vector(Vector3d::Zero());

    // Add box to model.

    math::RigidTransform<double> X_WA; // Identity
    if (FLAGS_rand_rpy) {
        srand(time(NULL));

        math::RollPitchYaw<double> R_WA(
            2.0*M_PI*double(rand()) / double(RAND_MAX),
            2.0*M_PI*double(rand()) / double(RAND_MAX),
            2.0*M_PI*double(rand()) / double(RAND_MAX));

        X_WA = math::RigidTransformd(R_WA, Vector3d::Zero());
    }

    // Add spatial inertia
    drake::multibody::SpatialInertia<double> M_Acm(
        FLAGS_box_m,
        Vector3d::Zero(),
        drake::multibody::UnitInertia<double>::SolidCube(FLAGS_box_l));

    const multibody::ModelInstanceIndex box_model_instance = plant.AddModelInstance("Box");
    const drake::multibody::RigidBody<double>& box = plant.AddRigidBody("Box", box_model_instance, M_Acm);

    // Visual Geometry
    plant.RegisterVisualGeometry(
        box, X_WA,
        geometry::Box(FLAGS_box_l, FLAGS_box_l, FLAGS_box_l),
        "BoxVisualGeometry",
        Eigen::Vector4d(0.5, 0.5, 0.5, 1));

    // Collision Geometry
    plant.RegisterCollisionGeometry(
        box, X_WA,
        geometry::Box(FLAGS_box_l, FLAGS_box_l, FLAGS_box_l),
        "BoxCollisionGeometry",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));

    // Now the model is complete.
    plant.Finalize();

    // Sanity check on the availability of the optional source id before using it.
    DRAKE_DEMAND(plant.geometry_source_is_registered());

    // drake::multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant);
    geometry::ConnectDrakeVisualizer(&builder, scene_graph);
    auto diagram = builder.Build();

    // Add EasyForce
    auto easy_force = builder.AddSystem(std::make_unique<EasyForce>(
        plant.GetBodyByName("Box").index(),
        Vector3d::Zero(),
        drake::multibody::SpatialForce<double>(
            Vector3d(FLAGS_Tx, FLAGS_Ty, FLAGS_Tz),
            Vector3d(FLAGS_Fx, FLAGS_Fy, FLAGS_Fz))));

    // connect the spatial force output port
    builder.Connect(
        easy_force->get_externally_applied_spatial_force_output_port(),
        plant.get_applied_spatial_force_input_port());

    // Create a context for this system:
    std::unique_ptr<systems::Context<double>> diagram_context =
        diagram->CreateDefaultContext();
    diagram->SetDefaultContext(diagram_context.get());
    systems::Context<double>& plant_context =
        diagram->GetMutableSubsystemContext(plant, diagram_context.get());
    plant.SetDefaultContext(&plant_context);

    systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
    auto& simulator_context = simulator.get_mutable_context();

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();

    // Using a while loop to step the sim as opposed to `simulator.StepTo(FLAGS_duration)` so that we
    // can periodically print the sim's context to command line for debugging.
    double current_time = 0.0;
    const double time_delta = 0.008;

    while( current_time < FLAGS_duration ){
        simulator.StepTo(current_time + time_delta);
        current_time = simulator_context.get_time();
    }

    return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple EasyForce box demo, pushing around a"
      "box to demonstrate EasyForce."
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::do_main();
}
