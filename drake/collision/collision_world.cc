#include "drake/collision/collision_world.h"

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/collision/bullet_collision_world.h"

using std::make_unique;
using std::move;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

CollisionWorld::CollisionWorld() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  bullet_pimpl_ = new BulletCollisionWorld();
}

CollisionWorld::~CollisionWorld() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  if(bullet_pimpl_) delete bullet_pimpl_;
}

CollisionElement* CollisionWorld::add_collision_element(
    std::unique_ptr<CollisionElement> e) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  CollisionElement* eptr = e.get();
  collision_elements_.push_back(move(e));
  return eptr;
}

void CollisionWorld::InitializeBulletCollisionElements() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  for(auto& element: collision_elements_) {
    element->bullet_pimpl_ =
        bullet_pimpl_->add_collision_element(
            make_unique<BulletCollisionElement>(
            element->geometry_, element->T_EG_));
  }
}

int CollisionWorld::get_num_elements() const {
  return collision_elements_.size();
}

int CollisionWorld::get_num_geometries() const {
  return collision_shapes_.size();
}

void CollisionWorld::Initialize() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  InitializeBulletCollisionElements();
}

void CollisionWorld::ClosestPointsAllToAll() {
}

void CollisionWorld::RayCast() {
}

}  // end namespace collision
}  // end namespace drake
