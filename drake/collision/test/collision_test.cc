#include "drake/collision/collision_world.h"
#include "drake/collision/collision_shape.h"

using std::make_unique;
using std::move;

using Eigen::Vector3d;
using Eigen::Isometry3d;
using drake::collision::CollisionElement;
using drake::collision::CollisionWorld;
using drake::collision::CollisionBox;
using drake::collision::CollisionSphere;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

// The collision engine API then looks like ...
int main() {

  CollisionWorld world;

  CollisionBox* box_shape =
      world.add_shape(
          make_unique<CollisionBox>(DrakeShapes::Box(Vector3d(1.0, 1.0, 1.0))));

  CollisionSphere* sphere_shape =
      world.add_shape(
          make_unique<CollisionSphere>(DrakeShapes::Sphere(1.0)));

  // Adds a collision elements using the previously defined shapes.
  CollisionElement* sphere =
      world.add_collision_element(
          make_unique<CollisionElement>(sphere_shape, Isometry3d::Identity()));

  // Adds two collision elements sharing the same box shape.
  CollisionElement* box1 =
      world.add_collision_element(
          make_unique<CollisionElement>(box_shape, Isometry3d::Identity()));

  CollisionElement* box2 =
      world.add_collision_element(
          make_unique<CollisionElement>(box_shape, Isometry3d::Identity()));

  (void) sphere;
  (void) box1;
  (void) box2;

  // From now on elements cannot be added.
  world.Initialize();

  PRINT_VAR(world.get_num_elements());
  PRINT_VAR(world.get_num_geometries());

  return 0;
}

