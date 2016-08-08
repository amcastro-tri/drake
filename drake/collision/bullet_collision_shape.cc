#include "drake/collision/bullet_collision_shape.h"

using Eigen::Vector3d;
using std::make_unique;
using std::move;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

BulletCollisionShape::~BulletCollisionShape() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

btCollisionShape* BulletCollisionShape::get_mutable_bullet_shape() {
  return bt_shape_.get();
}

BulletSphereShape::BulletSphereShape(double radius) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  bt_shape_.reset(new btSphereShape(radius));
}

BulletSphereShape::~BulletSphereShape() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

BulletBoxShape::BulletBoxShape(Vector3d size) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  // TODO(amcastro-tri): construct a convex hull instead as in
  // BulletModel::newBulletBoxShape.
  bt_shape_.reset(
      new btBoxShape(btVector3(size(0) / 2, size(1) / 2, size(2) / 2)));
}

BulletBoxShape::~BulletBoxShape() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

}  // end namespace collision
}  // end namespace drake