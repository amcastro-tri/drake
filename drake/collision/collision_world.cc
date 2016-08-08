#include "drake/collision/collision_world.h"

#include <memory>

#include "drake/collision/bullet_collision_world.h"

using Eigen::Isometry3d;
using std::make_unique;
using std::move;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

CollisionWorld::CollisionWorld() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  bullet_pimpl_.reset(new BulletCollisionWorld());
}

CollisionWorld::~CollisionWorld() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

CollisionElement* CollisionWorld::add_collision_element(
    std::unique_ptr<CollisionElement> e) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  CollisionElement* eptr = e.get();
  collision_elements_.push_back(move(e));
  return eptr;
}

CollisionElement* CollisionWorld::add_collision_element(
    const DrakeShapes::Sphere& sphere) {
  SphereCollisionShape* shape = add_collision_shape(
      make_unique<SphereCollisionShape>(sphere));
  return add_collision_element(
      make_unique<CollisionElement>(shape, Isometry3d::Identity()));
}

void CollisionWorld::InstantiateBulletImplementation() {
  PRINT_VAR(__PRETTY_FUNCTION__);

  for (auto& shape: collision_shapes_) {
    shape->bullet_pimpl_ =
        bullet_pimpl_->add_collision_shape(
            shape->InstantiateBulletImplementation());
  }

  for (auto& element: collision_elements_) {
    element->bullet_pimpl_ =
        bullet_pimpl_->add_collision_element(
            make_unique<BulletCollisionElement>(
            element->shape_->bullet_pimpl_, element->T_EG_));
  }
}

int CollisionWorld::get_num_elements() const {
  return collision_elements_.size();
}

int CollisionWorld::get_num_shapes() const {
  return collision_shapes_.size();
}

void CollisionWorld::Initialize() {
  PRINT_VAR(__PRETTY_FUNCTION__);
  InstantiateBulletImplementation();
}

void CollisionWorld::ClosestPointsAllToAll() {
}

void CollisionWorld::RayCast() {
}

}  // end namespace collision
}  // end namespace drake
