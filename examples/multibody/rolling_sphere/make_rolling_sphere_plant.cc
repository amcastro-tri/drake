#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"

#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {

using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using math::RigidTransformd;
using geometry::Sphere;
using geometry::HalfSpace;
using geometry::SceneGraph;

std::unique_ptr<drake::multibody::MultibodyPlant<double>> MakeBouncingBallPlant(
    double radius, double mass, double elastic_modulus, double dissipation,
    const CoulombFriction<double>& surface_friction,
    const Vector3<double>& gravity_W, SceneGraph<double>* scene_graph) {
  auto plant = std::make_unique<MultibodyPlant<double>>();

  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody("Ball", M_Bcm);

  if (scene_graph != nullptr) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);

    // TODO(SeanCurtis-TRI): Once SceneGraph supports hydroelastic contact
    //  between rigid half space and soft sphere, replace this box with the
    //  equivalent half space.
    const double size = 5;
    // If the box is positioned at (0, 0, z), the face's dividing edge passes
    // through the origin and, therefore, contact patch. Shift the box on the
    // z = 0 plane to make sure the intersection is a single triangle (so the
    // resolution of the contact surface is 100% defined by the sphere mesh.
    RigidTransformd X_WG{Vector3<double>(size / 10, -size / 10, -size / 2)};
    plant->RegisterCollisionGeometry(plant->world_body(), X_WG,
                                     geometry::Box(size, size, size),
                                     "collision", surface_friction);

    // Add visual for the ground.
    plant->RegisterVisualGeometry(plant->world_body(), X_WG,
                                  geometry::Box(size, size, size), "visual");

    // Add sphere geometry for the ball.
    // Pose of sphere geometry S in body frame B.
    const RigidTransformd X_BS = RigidTransformd::Identity();
    geometry::GeometryId sphere_geometry = plant->RegisterCollisionGeometry(
        ball, X_BS, Sphere(radius), "collision", surface_friction);

    // Set material properties for hydroelastics.
    plant->set_elastic_modulus(sphere_geometry, elastic_modulus);
    plant->set_hunt_crossley_dissipation(sphere_geometry, dissipation);

    // Add visual for the ball.
    const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
    plant->RegisterVisualGeometry(ball, X_BS, Sphere(radius), "visual", orange);

    // We add a few purple spots so that we can appreciate the sphere's
    // rotation.
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
    plant->RegisterVisualGeometry(ball, spot_pose({radial_offset, 0., 0.}),
                                  spot, "sphere_x+", red);
    plant->RegisterVisualGeometry(ball, spot_pose({-radial_offset, 0., 0.}),
                                  spot, "sphere_x-", red);
    plant->RegisterVisualGeometry(ball, spot_pose({0., radial_offset, 0.}),
                                  spot, "sphere_y+", green);
    plant->RegisterVisualGeometry(ball, spot_pose({0., -radial_offset, 0.}),
                                  spot, "sphere_y-", green);
    plant->RegisterVisualGeometry(ball, spot_pose({0., 0., radial_offset}),
                                  spot, "sphere_z+", blue);
    plant->RegisterVisualGeometry(ball, spot_pose({0., 0., -radial_offset}),
                                  spot, "sphere_z-", blue);
  }

  // Gravity acting in the -z direction.
  plant->mutable_gravity_field().set_gravity_vector(gravity_W);

  return plant;
}

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
