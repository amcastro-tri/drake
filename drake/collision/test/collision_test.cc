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
  DrakeShapes::Box* box_geometry =
      world.add_geometry(make_unique<DrakeShapes::Box>(Vector3d(1, 1, 1)));

  // Add a collision element using the previously created geometry.
  CollisionElement* box =
      world.add_collision_element(
          make_unique<CollisionElement>(box_geometry, Isometry3d::Identity()));

  // From now on elements cannot be added.
  world.Initialize();

  (void) box;

  PRINT_VAR(world.get_num_elements());
  PRINT_VAR(world.get_num_geometries());

  return 0;
}

