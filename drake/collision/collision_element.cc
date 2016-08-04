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
    DrakeShapes::Geometry* geometry,
    const Eigen::Isometry3d& T_EG): geometry_(geometry), T_EG_(T_EG) {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

CollisionElement::~CollisionElement() {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

CollisionElement* CollisionElement::New(
    CollisionWorld& world,
    DrakeShapes::Geometry &geometry,
    const Eigen::Isometry3d &T_EG) {
  PRINT_VAR(__PRETTY_FUNCTION__);
  auto owned_element = make_unique<CollisionElement>(&geometry, T_EG);
  CollisionElement* element = owned_element.get();
  world.add_collision_element(move(owned_element));
  return element;
}

void CollisionElement::set_geometry_to_element_transform(
    const Eigen::Isometry3d &T_EG) {
  if(bullet_pimpl_) {
    bullet_pimpl_->set_geometry_to_element_transform(T_EG);
    return;
  }
  // It only sets T_EG_ if no implementation was instantiated.
  T_EG_ = T_EG;
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