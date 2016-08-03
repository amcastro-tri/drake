#include "drake/collision/bullet_collision_world.h"

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.

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


int BulletCollisionWorld::get_number_of_elements() const {
  return collision_elements_.size();
}

BulletCollisionElement* BulletCollisionWorld::add_collision_element(
    std::unique_ptr<BulletCollisionElement> e) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  BulletCollisionElement* eptr = e.get();
  collision_elements_.push_back(move(e));
  return eptr;
}

void BulletCollisionWorld::Initialize() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

void BulletCollisionWorld::ClosestPointsAllToAll() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

void BulletCollisionWorld::RayCast() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

}  // end namespace collision
}  // end namespace drake
