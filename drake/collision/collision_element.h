#pragma once

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/systems/plants/shapes/DrakeShapes.h"
#include "drake/collision/collision_element_impl.h"

namespace drake {
namespace collision {

// Forward declaration to the collision world so that we can make it a friend of
// CollisionElement.
class CollisionWorld;

// This is the collision element instantiated by the user.
// The actual implementation (Bullet, FCL, others) is completely hidden.
// There are no Bullet or FCL includes here.

//class DRAKECOLLISIONENGINE_EXPORT CollisionElement {
// Handle class. Should this "handle" description be part of the doxygen doc?
class CollisionElement {
  // CollisionWorld has access to CollisionElement's internal implementation.
  // In particular, it has access to its pimpl_.
  friend class CollisionWorld;
 public:
  /**
  @param geometry[in] The geometric model of the collision element.
  @param T_EG[in] pose of the element's geometry in the frame of
  the element. In other words, a transformation from the geometry's frame into the
  collision element's frame.**/
  CollisionElement(
      const DrakeShapes::Geometry &geometry,
      const Eigen::Isometry3d &T_EG);

  ~CollisionElement();

  static CollisionElement* New(
      CollisionWorld& world,
      const DrakeShapes::Geometry &geometry,
      const Eigen::Isometry3d &T_EG);

#if 0
  void update_geometry_to_element_transform(const Eigen::Isometry3d &T) {
    pimpl_->update_geometry_to_element_transform(T);
  }

  // Factory method provided so that users do not ever see a "new".
  // CollsionWorld here is nothing but DrakeCollision::Model
  // (the "model" world is used too much in drake and this change won't brake any API)
  static CollisionElement *CreateAndAddToCollisionWorld(
      const CollisionWorld &world,
      const DrakeShapes::Geometry &geometry,
      const Eigen::Isometry3d &T_geo_to_element);
#endif

  void update_geometry_to_element_transform(
      const Eigen::Isometry3d &T_EG);

 private:
  // The underlying back end to CollisionElement.
  // The collision element owns its implementation.
  std::unique_ptr<CollisionElementImpl> pimpl_{nullptr};
};

}  // end namespace collision
}  // end namespace drake
