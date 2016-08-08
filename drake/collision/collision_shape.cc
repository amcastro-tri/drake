#include "drake/collision/collision_shape.h"

#include "drake/collision/collision_shape.h"
#include "drake/collision/bullet_collision_shape.h"

using std::make_unique;
using std::move;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

CollisionShape::~CollisionShape() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

CollisionSphere::CollisionSphere(const DrakeShapes::Sphere& s) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  radius_ = s.radius;
}

CollisionSphere::~CollisionSphere() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

std::unique_ptr<class BulletCollisionShape>
CollisionSphere::InstantiateBulletImplementation() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  auto owned_pimpl = make_unique<BulletCollisionSphere>(radius_);
  bullet_pimpl_ = owned_pimpl.get();
  return move(owned_pimpl);
}

CollisionBox::CollisionBox(const DrakeShapes::Box& box): size_(box.size) {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

CollisionBox::~CollisionBox() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

std::unique_ptr<class BulletCollisionShape>
CollisionBox::InstantiateBulletImplementation() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  auto owned_pimpl = make_unique<BulletCollisionBox>(size_);
  bullet_pimpl_ = owned_pimpl.get();
  return move(owned_pimpl);
}

}  // end namespace collision
}  // end namespace drake