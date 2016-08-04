#include "collision_world_impl.h"

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/collision/bullet_collision_element.h"
#include "drake/collision/collision_element_impl.h"

using std::make_unique;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

CollisionWorldImpl::CollisionWorldImpl() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

CollisionWorldImpl::~CollisionWorldImpl() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

CollisionElementImpl* CollisionWorldImpl::add_collision_element(
    CollisionElementImpl* e) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  collision_elements_.push_back(e);
  return e;
}

DrakeShapes::Geometry* CollisionWorldImpl::add_geometry(
    std::unique_ptr<DrakeShapes::Geometry> g) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  DrakeShapes::Geometry* gptr = g.get();
  geometries_.push_back(move(g));
  return gptr;
}

int CollisionWorldImpl::get_num_elements() const {
  return collision_elements_.size();
}

int CollisionWorldImpl::get_num_geometries() const {
  return geometries_.size();
}

void CollisionWorldImpl::Initialize() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  bullet_pimpl_.reset(new BulletCollisionWorld());
}

#if 0
void CollisionWorldImpl::InitializeBulletWorld() {
  auto owned_bullet_element = make_unique<BulletCollisionElement>()
  auto bullet_element = owned_bullet_element.get();
  bullet_pimpl_->add_collision_element(owned_bullet_element);
}
#endif

void CollisionWorldImpl::ClosestPointsAllToAll() {

}

void CollisionWorldImpl::RayCast() {

}

}  // end namespace collision
}  // end namespace drake
