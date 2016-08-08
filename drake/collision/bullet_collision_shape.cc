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

BulletCollisionSphere::BulletCollisionSphere(double radius) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  bt_shape_.reset(new btSphereShape(radius));
}

BulletCollisionSphere::~BulletCollisionSphere() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

BulletCollisionBox::BulletCollisionBox(Vector3d size) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  // TODO(amcastro-tri): construct a convex hull instead as in
  // BulletModel::newBulletBoxShape.
  bt_shape_.reset(
      new btBoxShape(btVector3(size(0) / 2, size(1) / 2, size(2) / 2)));
}

BulletCollisionBox::~BulletCollisionBox() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

}  // end namespace collision
}  // end namespace drake