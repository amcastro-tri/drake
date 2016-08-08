#include "drake/collision/bullet_collision_world.h"

#include <memory>

#include <Eigen/Dense>

using std::make_unique;
using std::move;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

BulletCollisionWorld::BulletCollisionWorld() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

BulletCollisionWorld::~BulletCollisionWorld() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}


int BulletCollisionWorld::get_num_elements() const {
  return collision_elements_.size();
}

BulletCollisionShape* BulletCollisionWorld::add_collision_shape(
    std::unique_ptr<BulletCollisionShape> shape) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  BulletCollisionShape* shape_ptr = shape.get();
  collision_shapes_.push_back(move(shape));
  return shape_ptr;
}

BulletCollisionElement* BulletCollisionWorld::add_collision_element(
    std::unique_ptr<BulletCollisionElement> element) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  BulletCollisionElement* element_ptr = element.get();
  collision_elements_.push_back(move(element));
  return element_ptr;
}

void BulletCollisionWorld::ClosestPointsAllToAll() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

void BulletCollisionWorld::RayCast() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

}  // end namespace collision
}  // end namespace drake
