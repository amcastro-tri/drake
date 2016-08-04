#pragma once

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/systems/plants/shapes/DrakeShapes.h"

namespace drake {
namespace collision {

// This is the collision element instantiated by the user.
// The actual implementation (Bullet, FCL, others) is completely hidden.
// There are no Bullet or FCL includes here.

//class DRAKECOLLISIONENGINE_EXPORT CollisionElement {
// Handle class. Should this "handle" description be part of the doxygen doc?
class CollisionElement {
  // CollisionWorld has access to CollisionElement's internal implementation.
  // In particular, it has access to its pimpl's.
  friend class CollisionWorld;
 public:
  /**
  @param geometry[in] The geometric model of the collision element.
  @param T_EG[in] pose of the element's geometry in the frame of
  the element. In other words, a transformation from the geometry's frame into the
  collision element's frame.**/
  CollisionElement(
      DrakeShapes::Geometry* geometry,
      const Eigen::Isometry3d &T_EG);

  ~CollisionElement();

  static CollisionElement* New(
      CollisionWorld& world,
      DrakeShapes::Geometry &geometry,
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

  void set_geometry_to_element_transform(
      const Eigen::Isometry3d &T_EG);

 private:
  // The underlying back end to CollisionElement.
  // The collision element does not own its implementation, the appropriate
  // implementation does.
  // For instance, BulletCollisionWorld owns its BulletCollisionElement's.
  class BulletCollisionElement* bullet_pimpl_{nullptr};

  // The geometry of this collision element. It is referenced by a non-owning
  // raw pointer. The list of geometries is managed by the CollisionWorld.
  DrakeShapes::Geometry* geometry_{nullptr};

  // Geometry's frame to element's frame transformation. This is only used
  // before implementations are initialized as a temporary copy for when
  // implementations are instantiated.
  Eigen::Isometry3d T_EG_;
};

}  // end namespace collision
}  // end namespace drake
