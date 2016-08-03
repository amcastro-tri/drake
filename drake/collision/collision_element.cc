#include "collision_element.h"

#include "collision_element_impl.h"
#include "drake/common/drake_assert.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

CollisionElement::CollisionElement(
    const DrakeShapes::Geometry& geometry,
    const Eigen::Isometry3d& T_EG) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  pimpl_.reset(new CollisionElementImpl(geometry, T_EG));
}

CollisionElement::~CollisionElement() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

void CollisionElement::update_geometry_to_element_transform(
    const Eigen::Isometry3d &T_EG) {
  pimpl_->update_geometry_to_element_transform(T_EG);
}

#if 0
CollisionElement* CollisionElement::CreatedAndAddToCollisionWorld(const CollisionWorld& world, const DrakeShapes::Geometry& geometry, const Eigen::Isometry3d& T_geo_to_element) {
  CollisionElement* element{nullptr};
  std::unique_ptr<CollisionElement> owned_element(element = new CollisionElement(geometry, T_geo_to_element));
  world.add_collision_element(owned_element);
  return element; // Returns a pointer in case the user wants to do something like element->update_transform(T) or element->set_static(), no id's!!
}
#endif

}  // end namespace collision
}  // end namespace drake