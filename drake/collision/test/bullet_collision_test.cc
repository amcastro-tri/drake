#include <memory>

//#include "drake/collision/bullet_collision_element.h"
#include "drake/collision/bullet_collision_world.h"

using std::make_unique;
using std::move;

using Eigen::Vector3d;
using Eigen::Isometry3d;
using drake::collision::BulletCollisionElement;
using drake::collision::BulletCollisionWorld;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;


// The collision engine API then looks like ...
int main() {

  BulletCollisionWorld world;

  BulletCollisionElement* box =
      world.add_collision_element(
          make_unique<BulletCollisionElement>(
              DrakeShapes::Box(Vector3d(1, 1, 1)) /* geometry */,
              Isometry3d::Identity() /* geometry to element transformation */));

  (void) box;

  PRINT_VAR(world.get_number_of_elements());
  
  return 0;
}

