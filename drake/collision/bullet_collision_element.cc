#include "drake/collision/bullet_collision_element.h"

#include <memory>

using std::make_unique;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

BulletCollisionElement::BulletCollisionElement(
    BulletCollisionShape* shape,
    const Eigen::Isometry3d& T_geo_to_element) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  bt_collision_object_ = make_unique<btCollisionObject>();

  // Sets margins.
  //bt_shape = newBulletSphereShape(sphere, true);
  //bt_shape->setMargin(kLargeMargin);

  // This Bullet's method requires a non-const pointer to shape.
  bt_collision_object_->setCollisionShape(shape->get_mutable_bullet_shape());

  // Bullet never uses this pointer internally.
  // This pointer can be retrieved with btCollisionObject::getUserPointer().
  bt_collision_object_->setUserPointer(this);
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
