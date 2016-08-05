#include "drake/collision/bullet_collision_element.h"

#include <memory>

using std::make_unique;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

BulletCollisionElement::BulletCollisionElement(
    const DrakeShapes::Geometry* geometry,
    const Eigen::Isometry3d& T_geo_to_element) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  bt_collision_object_ = make_unique<btCollisionObject>();
  // ... Here either use something similar to what happens right now in BulletModel::addElement (nasty) ...
  // ... or use double dispatch to get the right derived Bullet object created ..
  //bt_obj_ = /*...*/
}

BulletCollisionElement::~BulletCollisionElement() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

void BulletCollisionElement::set_geometry_to_element_transform(
    const Eigen::Isometry3d &T) {
  //btTransform btT;
  // .. code to go from Eigen T to Bullet's btT
  //bt_obj_->setWorldTransform(btT);
}

}  // end namespace collision
}  // end namespace drake
