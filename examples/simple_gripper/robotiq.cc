#include <iostream>
#include <memory>
#include <string>

#include <fmt/format.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/visualization/visualization_config_functions.h"
namespace drake {
namespace examples {
namespace simple_gripper {
namespace {

int do_main() {
  auto meshcat = std::make_shared<geometry::Meshcat>();
  systems::DiagramBuilder<double> builder;

  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.002);
  plant.set_discrete_contact_solver(multibody::DiscreteContactSolver::kSap);

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

  plant.Finalize();

  auto torque = builder.AddSystem<systems::ConstantVectorSource>(Vector1d(1));
  builder.Connect(torque->get_output_port(), plant.get_actuation_input_port());

  visualization::AddDefaultVisualization(&builder, meshcat);

  auto diagram = builder.Build();

  // Set up simulator.
  systems::Simulator simulator(*diagram);

  meshcat->StartRecording(32.0, false);
  simulator.AdvanceTo(20.0);
  meshcat->PublishRecording();

  // Pause so that you can see the meshcat output.
  std::cout << "[Press Ctrl-C to finish]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  return 0;
}

}  // namespace
}  // namespace simple_gripper
}  // namespace examples
}  // namespace drake

int main(int, char*[]) {
  return drake::examples::simple_gripper::do_main();
}
