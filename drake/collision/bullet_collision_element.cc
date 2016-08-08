#include "drake/collision/bullet_collision_element.h"

#include <memory>

using std::make_unique;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

BulletCollisionElement::BulletCollisionElement(
    const BulletCollisionShape* shape,
    const Eigen::Isometry3d& T_geo_to_element) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  bt_collision_object_ = make_unique<btCollisionObject>();

#if 0
  // Create shape.
  BulletCollisionShapeMaker maker; // Inherits from abstract ShapeMaker (with virtual method Make() = 0)
  CollisionShape* shape = geometry->MakeShape(maker);
  // Transfer ownership of the collision shape to the geometry.
  geometry->set_user_data(unique_ptr<CollisionShape>(shape));

  // Since the maker is of type BulletCollisionShapeMaker, it is safe to cast to
  // a BulletCollisionShape here.
  btCollisionShape* bt_shape =
      dynamic_cast<BulletCollisionShape*>(shape)->bt_shape_;

  // This will go inside the method BulletCollisionShapeMaker::MakeShape(Sphere& s)
  //bt_shape = newBulletSphereShape(sphere, true);
  //bt_shape->setMargin(kLargeMargin);

  bt_collision_object_->setCollisionShape(bt_shape);

  // Bullet never uses this pointer internally.
  // This pointer can be retrieved with btCollisionObject::getUserPointer().
  bt_collision_object_->setUserPointer(this);
#endif
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
