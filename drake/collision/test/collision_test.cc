#include "drake/collision/collision_world.h"

using std::make_unique;
using std::move;

using Eigen::Vector3d;
using Eigen::Isometry3d;
using drake::collision::CollisionElement;

// The collision engine API then looks like ...
int main() {

  CollisionWorld world;

  DrakeShapes::Box box(Vector3d(1, 1, 1));
  //CollisionElement* colliding_box = world.create_collision_element(box, T_EG, mat);
  // Collision element is very light weight.
  // It does contain a vector of cliques and a transform matrix, but that's it.
  CollisionElement* colliding_box = world.add_collision_element(
      make_unique<CollisionElement>(box, T_EG, mat));




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

