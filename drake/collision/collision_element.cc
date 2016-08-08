#include "collision_element.h"

#include "drake/collision/bullet_collision_element.h"
#include "drake/collision/collision_world.h"

using std::make_unique;
using std::move;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

CollisionElement::CollisionElement(
    const CollisionShape* shape,
    const Eigen::Isometry3d& T_EG): shape_(shape), T_EG_(T_EG) {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

CollisionElement::~CollisionElement() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

void CollisionElement::set_geometry_to_element_transform(
    const Eigen::Isometry3d &T_EG) {
  if(bullet_pimpl_) {
    bullet_pimpl_->set_geometry_to_element_transform(T_EG);
    return;
  }
  // It only sets T_EG_ if no implementation was instantiated and uses its value
  // when instantiating the implementation.
  T_EG_ = T_EG;
}

}  // end namespace collision
}  // end namespace drake