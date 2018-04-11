#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_plant.h"

#include "drake/common/find_resource.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using geometry::Cylinder;
using geometry::HalfSpace;
using geometry::Mesh;
using geometry::FrameId;
using geometry::GeometrySystem;
using geometry::Sphere;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::RotationalInertia;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeKukaIiwaPlant(bool finalize,
                  geometry::GeometrySystem<double>* geometry_system) {
  auto plant = std::make_unique<MultibodyPlant<double>>(
      MakeKukaIiwaModel<double>(false /*do no finalize model yet*/));

  const Body<double>& link1 = plant->model().GetBodyByName("iiwa_link_1");
  const Body<double>& link2 = plant->model().GetBodyByName("iiwa_link_2");
  const Body<double>& link3 = plant->model().GetBodyByName("iiwa_link_3");
  const Body<double>& link4 = plant->model().GetBodyByName("iiwa_link_4");
  const Body<double>& link5 = plant->model().GetBodyByName("iiwa_link_5");
  const Body<double>& link6 = plant->model().GetBodyByName("iiwa_link_6");
  const Body<double>& link7 = plant->model().GetBodyByName("iiwa_link_7");

  // Add geometry for visualization.
  plant->RegisterAsSourceForGeometrySystem(geometry_system);

  // A half-space for the ground geometry.
  std::string mesh_path;
  mesh_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/meshes/visual/link_0.obj");
  plant->RegisterCollisionGeometry(
      plant->world_body(), Isometry3d::Identity(), Mesh(mesh_path),
      geometry_system);

  mesh_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/meshes/visual/link_1.obj");
  plant->RegisterVisualGeometry(
      link1, Isometry3d::Identity(), Mesh(mesh_path), geometry_system);

  mesh_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/meshes/visual/link_2.obj");
  plant->RegisterVisualGeometry(
      link2, Isometry3d::Identity(), Mesh(mesh_path), geometry_system);

  mesh_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/meshes/visual/link_3.obj");
  plant->RegisterVisualGeometry(
      link3, Isometry3d::Identity(), Mesh(mesh_path), geometry_system);

  mesh_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/meshes/visual/link_4.obj");
  plant->RegisterVisualGeometry(
      link4, Isometry3d::Identity(), Mesh(mesh_path), geometry_system);

  mesh_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/meshes/visual/link_5.obj");
  plant->RegisterVisualGeometry(
      link5, Isometry3d::Identity(), Mesh(mesh_path), geometry_system);

  mesh_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/meshes/visual/link_6.obj");
  plant->RegisterVisualGeometry(
      link6, Isometry3d::Identity(), Mesh(mesh_path), geometry_system);

  mesh_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/meshes/visual/link_7.obj");
  plant->RegisterVisualGeometry(
      link7, Isometry3d::Identity(), Mesh(mesh_path), geometry_system);

  // We are done creating the plant.
  if (finalize) plant->Finalize();

  return plant;
}

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
