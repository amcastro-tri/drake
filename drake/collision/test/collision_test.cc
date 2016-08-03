#include "drake/collision/collision_world.h"

using std::make_unique;
using std::move;

using Eigen::Vector3d;
using Eigen::Isometry3d;
using drake::collision::CollisionElement;
using drake::collision::CollisionWorld;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

// The collision engine API then looks like ...
int main() {

  CollisionWorld world;

  // Create a new box and add it to the collision world.
  auto box_geometry = DrakeShapes::Box::New(world, Vector3d(1, 1, 1));

  // Add a collision element using the previously created geometry.
  //auto box = CollisionElement::New(world, box_geometry, Isometry3d::Identity());

  // From now on elements cannot be added.
  world.Initialize();

  (void) box_geometry;

  PRINT_VAR(world.get_number_of_elements());

  return 0;
}

