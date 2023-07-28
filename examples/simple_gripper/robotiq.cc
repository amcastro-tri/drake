#include <iostream>
#include <memory>
#include <string>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/common/fmt_eigen.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/systems/analysis/simulator_print_stats.h"

using drake::systems::PrintSimulatorStatistics;
using drake::multibody::ContactResults;
using drake::multibody::ContactModel;
using drake::geometry::CollisionFilterDeclaration;
using drake::geometry::GeometrySet;

DEFINE_double(simulation_time, 2.0, "Simulation duration in seconds");

namespace drake {
namespace examples {
namespace simple_gripper {
namespace {

int do_main() {
  auto meshcat = std::make_shared<geometry::Meshcat>();
  systems::DiagramBuilder<double> builder;

  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.01);
  plant.set_discrete_contact_solver(multibody::DiscreteContactSolver::kSap);
  //plant.set_contact_model(ContactModel::kHydroelastic);

  multibody::Parser parser(&plant);
  multibody::PackageMap::RemoteParams params;
  params.urls = {"https://github.com/RussTedrake/kinova-movo/archive/"
           "d94d1d7da7ff8fc71f2439bb0a8989f1e6fd79b4.tar.gz"};
  params.sha256 =
      "a9201477a23f410f10d00e86847de778c175d3d3c8971be52a9ac881194e4887";
  params.strip_prefix = "kinova-movo-d94d1d7da7ff8fc71f2439bb0a8989f1e6fd79b4";
  //parser.package_map().AddRemote("kinova-movo", params);
  parser.package_map().AddPackageXml(
      "/home/amcastro/Documents/Drake/kinova-movo/movo_common/movo_description/"
      "package.xml");
  //parser.package_map().AddPackageXml(
  //    parser.package_map().GetPath("kinova-movo") +
  //    "/movo_common/movo_description/package.xml");

  // Print path to see where files end up.
  //std::cout << fmt::format(" Path: {}\n",
  //                         parser.package_map().GetPath("kinova-movo"));

  // /home/amcastro/.cache/drake/package_map/a9201477a23f410f10d00e86847de778c175d3d3c8971be52a9ac881194e4887-660e65fbc966cbcbcef5a5b4a87cd8a4d93a4e88bba0b78d4d6495d95e119b22/movo_common/movo_description/urdf/manipulation/robotiq/robotiq_85_gripper.urdf
  // /home/amcastro/.cache/drake/package_map/a9201477a23f410f10d00e86847de778c175d3d3c8971be52a9ac881194e4887-660e65fbc966cbcbcef5a5b4a87cd8a4d93a4e88bba0b78d4d6495d95e119b22/movo_common/movo_description/urdf/manipulation/robotiq/robotiq_85_gripper.urdf.xacro

  std::string with_mimic = R"""(
directives:
- add_model:
    name: robotiq
    file: package://movo_description/urdf/manipulation/robotiq/robotiq_85_gripper.urdf

- add_weld:
    parent: world
    child: robotiq::robotiq_coupler_link
    X_PC:
        rotation: !Rpy { deg: [90.0, 0.0, -90.0 ]}

- add_model:
    name: spam
    file: package://drake/manipulation/models/ycb/sdf/010_potted_meat_can.sdf
    default_free_body_pose: { base_link_meat: { 
        translation: [0.2, 0.0, 0.0],
        rotation: !Rpy { deg: [-90.0, 0.0, 0.0 ]}
    } }

- add_model:
    name: table
    file: package://drake/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table_surface_only_collision.sdf
    
- add_weld:
    parent: world
    child: table::link
    X_PC:
        translation: [0.0, 0.0, -0.81]
)""";

  parser.AddModelsFromString(with_mimic, "dmd.yaml");

  auto get_gid = [&](const std::string& body_name,
                     const std::string& geo_name) {
    const auto frame_id = plant.GetBodyFrameIdOrThrow(
        plant.GetRigidBodyByName(body_name).index());
    const auto geo_id = scene_graph.model_inspector().GetGeometryIdByName(
        frame_id, geometry::Role::kProximity, geo_name);
    return geo_id;
  };
  (void)get_gid;

  //auto meat_box_geo = get_gid("base_link_meat", "spam::box_collision");
  //auto ground_geo = get_gid("link", "table::surface");
  //scene_graph.collision_filter_manager().Apply(
  //    CollisionFilterDeclaration().ExcludeBetween(GeometrySet(meat_box_geo), GeometrySet(ground_geo)));

  plant.Finalize();

  auto torque = builder.AddSystem<systems::ConstantVectorSource>(Vector1d(1));
  builder.Connect(torque->get_output_port(), plant.get_actuation_input_port());

  visualization::AddDefaultVisualization(&builder, meshcat);

  auto diagram = builder.Build();

  // Set up simulator.
  systems::Simulator simulator(*diagram);

  auto monitor = [&plant](const systems::Context<double>& diagram_context) {
    const auto& context = plant.GetMyContextFromRoot(diagram_context);
    const auto& contacts_port = plant.get_contact_results_output_port();
    const ContactResults<double>& results =
        contacts_port.Eval<ContactResults<double>>(context);

    std::cout << std::string(80, '*') << std::endl;        
    std::cout << fmt::format(
        " Time: {}. num_points = {}. num_hydro = {}.\n", context.get_time(),
        results.num_point_pair_contacts(), results.num_hydroelastic_contacts());
    for (int i = 0; i < results.num_point_pair_contacts(); ++i) {
      const auto& pp = results.point_pair_contact_info(i);
      const std::string bodyA = plant.get_body(pp.bodyA_index()).name();
      const std::string bodyB = plant.get_body(pp.bodyB_index()).name();
      const std::string finger2 = "gripper_finger2_finger_tip_link";
      if (bodyA == finger2 || bodyB == finger2) {
        std::cout << fmt::format("{}: A = {}. B = {}\n", i, bodyA, bodyB);
        std::cout << fmt::format("  p_WC = {}\n",
                                 fmt_eigen(pp.contact_point().transpose()));
        std::cout << fmt::format(
            "  n_W  = {}\n", fmt_eigen(pp.point_pair().nhat_BA_W.transpose()));
      }
    }
    return systems::EventStatus::Succeeded();
  };

  simulator.set_monitor(monitor);

  meshcat->StartRecording(32.0, false);
  simulator.AdvanceTo(FLAGS_simulation_time);
  meshcat->PublishRecording();
  PrintSimulatorStatistics(simulator);
  // Pause so that you can see the meshcat output.
  std::cout << "[Press Ctrl-C to finish]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  return 0;
}

}  // namespace
}  // namespace simple_gripper
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::simple_gripper::do_main();
}
