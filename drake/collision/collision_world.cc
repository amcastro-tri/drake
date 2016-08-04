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

CollisionElement* CollisionWorld::add_collision_element(
    std::unique_ptr<CollisionElement> e) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  CollisionElement* eptr = e.get();
  collision_elements_.push_back(move(e));
  // The implementation keeps a non-owning reference.
  pimpl_->add_collision_element(
      dynamic_cast<CollisionElementImpl*>(eptr->pimpl_.get()));
  return eptr;
}

DrakeShapes::Geometry* CollisionWorld::add_geometry(
    std::unique_ptr<DrakeShapes::Geometry> g) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  return pimpl_->add_geometry(move(g));
}

int CollisionWorld::get_num_elements() const {
  return pimpl_->get_num_elements();
}

int CollisionWorld::get_num_geometries() const {
  return pimpl_->get_num_geometries();
}

void CollisionWorld::Initialize() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  pimpl_->Initialize();
}

void CollisionWorld::ClosestPointsAllToAll() {
  pimpl_->ClosestPointsAllToAll();
}

void CollisionWorld::RayCast() {
  pimpl_->RayCast();
}

}  // end namespace collision
}  // end namespace drake
