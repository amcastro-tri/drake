#include "drake/collision/collision_world.h"
#include "drake/collision/collision_shape.h"

using std::make_unique;
using std::move;

using Eigen::Vector3d;
using Eigen::Isometry3d;
using drake::collision::CollisionElement;
using drake::collision::CollisionWorld;
using drake::collision::CollisionShape;
using drake::collision::BoxCollisionShape;
using drake::collision::SphereCollisionShape;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

// The collision engine API then looks like ...
int main() {

  CollisionWorld world;

  // Creates a sphere collision element.
  CollisionElement* sphere =
      world.add_collision_element(DrakeShapes::Sphere(1.0));
  sphere->set_geometry_to_element_transform(Isometry3d::Identity());

  // Create a sphere shape and share it across multiple collision elements.
  CollisionShape* box_shape =
      world.add_collision_shape(
          make_unique<BoxCollisionShape>(DrakeShapes::Box(Vector3d(1.0, 1.0, 1.0))));

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
  PRINT_VAR(world.get_num_shapes());

  return 0;
}

