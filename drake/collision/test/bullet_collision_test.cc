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

  /*
  // or you could've created like this instead:
  CollisionElement* colliding_box = world->add_collision_element(
   make_unique<CollisionElement>(box, Isometry3d::Identity()));

  // The user still can perform actions on collision elements after they are
  // added to the collision world.
  colliding_box->update_geometry_to_element_transform(Isometry3d::Identity());
  colliding_box->join_clique(3);
*/

  // No nasty id's needed to access my collision element!
  //colliding_box->add_to_collision_clique(3);

  // Cleans up properly.
  return 0;
}

