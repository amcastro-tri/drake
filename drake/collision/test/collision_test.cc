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

  CollisionElement* box = world.add_collision_element(
      make_unique<CollisionElement>(
          DrakeShapes::Box(Vector3d(1, 1, 1)) /* geometry */,
          Isometry3d::Identity() /* geometry to element transformation */));

  // From now on elements cannot be added.
  world.Initialize();

  (void) box;

  PRINT_VAR(world.get_number_of_elements());

  return 0;
}

