#include "collision_world_impl.h"

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/collision/bullet_collision_element.h"
#include "drake/collision/collision_element_impl.h"

using std::make_unique;

namespace drake {
namespace collision {

void CollisionWorldImpl::Initialize() {

}

CollisionElement* CollisionWorldImpl::add_collision_element(
    std::unique_ptr<CollisionElement> e) {
  if (bullet_pimpl_) {
    BulletCollisionElement *bce = bullet_pimpl_->add_collision_element(
        make_unique<BulletCollisionElement>(
            e->get_geometry(), e->get_geometry_to_element_transform()));
    static_cast<CollisionElementImpl*>(e.get()->pimpl_)->bullet_pimpl_ = bce;
  } else {
    throw std::runtime_error(
        "No implementation instantiated for CollisionWorld.");
  }
}

void CollisionWorldImpl::ClosestPointsAllToAll() {

}

void CollisionWorldImpl::RayCast() {

}

}  // end namespace collision
}  // end namespace drake
