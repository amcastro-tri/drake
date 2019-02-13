#include "drake/examples/multibody/volumetric_contact_demo/make_cylinder_plant.h"

#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cylinder_with_multicontact {

using geometry::Cylinder;
using geometry::HalfSpace;
using geometry::SceneGraph;
using geometry::Sphere;
using geometry::Mesh;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;
using Eigen::Isometry3d;

void AddGeometry(
    MultibodyPlant<double>* plant, const RigidBody<double>& body,
    double length, const CoulombFriction<double>& friction) {
  const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  const Vector4<double> red(1.0, 0.0, 0.0, 1.0);

  const std::string model_file = "drake/examples/multibody/volumetric_contact_demo/cube.obj";
  const std::string obj_model = FindResourceOrThrow(model_file);

  // Visual for the Cylinder
  plant->RegisterVisualGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the cylinder body frame B. */
      Isometry3d::Identity(), Mesh(obj_model, length), "visual", orange);
      
  plant->RegisterMeshGeometry(
        body,
        Isometry3d::Identity(),
        obj_model, "collision",
        friction, Vector3<double>(length, length, length));

#if 0
  // Add a bunch of little spheres to simulate "multi-contact".
  for (int i = 0; i < num_contacts; ++i) {
    const double theta = 2.0 * i / num_contacts * M_PI;
    const double x = cos(theta) * radius;
    const double y = sin(theta) * radius;
    Isometry3<double> X_BG = Isometry3<double>::Identity();
    // Top spheres:
    /* Pose X_BG of the geometry frame G in the cylinder body frame B. */
    X_BG.translation() << x, y, length / 2;
    plant->RegisterCollisionGeometry(
        body,
        X_BG,
        Sphere(contact_spheres_radius), "collision_top_" + std::to_string(i),
        friction);
    plant->RegisterVisualGeometry(
        body,
        X_BG,
        Sphere(contact_spheres_radius), "visual_top_" + std::to_string(i), red);

    // Bottom spheres:
    X_BG.translation() << x, y, -length / 2;
    plant->RegisterCollisionGeometry(
        body,
        X_BG,
        Sphere(contact_spheres_radius), "collision_bottom_" + std::to_string(i),
        friction);
    plant->RegisterVisualGeometry(
        body,
        X_BG,
        Sphere(contact_spheres_radius), "visual_bottom_" + std::to_string(i),
        red);
  }
  #endif
}

std::unique_ptr<drake::multibody::MultibodyPlant<double>>
MakeCylinderPlant(double length, double mass,
                  const CoulombFriction<double>& surface_friction,
                  const Vector3<double>& gravity_W, double dt,
                  geometry::SceneGraph<double>* scene_graph) {
  DRAKE_DEMAND(scene_graph != nullptr);

  auto plant = std::make_unique<MultibodyPlant<double>>(dt);
  plant->RegisterAsSourceForSceneGraph(scene_graph);

  UnitInertia<double> G_Bcm =
      UnitInertia<double>::SolidCube(length);

  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);
  const RigidBody<double>& cylinder = plant->AddRigidBody("Cylinder", M_Bcm);

  // Add geometry to the cylinder for both contact and visualization.
  AddGeometry(
      plant.get(),
      cylinder, length, surface_friction);

  // Add a model for the ground.
  Vector3<double> normal_W(0, 0, 1);
  Vector3<double> point_W(0, 0, 0);

  // A half-space for the ground geometry.
  const std::string model_file = "drake/examples/multibody/volumetric_contact_demo/ground_box.obj";
  const std::string obj_model = FindResourceOrThrow(model_file);
  plant->RegisterMeshGeometry(
        plant->world_body(),
        HalfSpace::MakePose(normal_W, point_W),
        obj_model, "collision",
        surface_friction, Vector3<double>(1.0, 1.0, 1.0));

  // Add visual for the ground.
  plant->RegisterVisualGeometry(
      plant->world_body(), HalfSpace::MakePose(normal_W, point_W),
      Mesh(obj_model), "visual");

  plant->AddForceElement<UniformGravityFieldElement>(gravity_W);

  // We are done creating the plant.
  plant->Finalize();

  return plant;
}

}  // namespace cylinder_with_multicontact
}  // namespace multibody
}  // namespace examples
}  // namespace drake
