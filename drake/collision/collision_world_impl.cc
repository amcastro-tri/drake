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

int CollisionWorldImpl::get_number_of_elements() const {
  return bullet_pimpl_->get_number_of_elements();
}

void CollisionWorldImpl::Initialize() {

}

void CollisionWorldImpl::ClosestPointsAllToAll() {

}

void CollisionWorldImpl::RayCast() {

}

}  // end namespace collision
}  // end namespace drake
