#include "drake/collision/collision_world.h"

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/collision/collision_world_impl.h"

using std::make_unique;
using std::move;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

CollisionWorld::CollisionWorld() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  pimpl_.reset(new CollisionWorldImpl());
}

CollisionWorld::~CollisionWorld() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

void CollisionWorld::Initialize() {
  pimpl_->Initialize();
}

CollisionElement* CollisionWorld::add_collision_element(
    std::unique_ptr<CollisionElement> e) {
  CollisionElement* eptr = e.get();
  collision_elements_.push_back(move(e));
  return eptr;
}

void CollisionWorld::ClosestPointsAllToAll() {
  pimpl_->ClosestPointsAllToAll();
}

void CollisionWorld::RayCast() {
  pimpl_->RayCast();
}

}  // end namespace collision
}  // end namespace drake
