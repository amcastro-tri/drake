#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/drakeCollisionEngine_export.h"
#include "drake/systems/plants/shapes/DrakeShapes.h"

namespace drake {
namespace collision {

/** An element of a CollisionWorld representing a geometry with a given pose
in the world. */
// This is a handle class to its internal implementations (PIMPL's).
// Currently available implementations include: Bullet.
class DRAKECOLLISIONENGINE_EXPORT CollisionElement {
  // CollisionWorld has access to CollisionElement's internal implementation.
  // In particular, it has access to its pimpl's.
  friend class CollisionWorld;
 public:
  /**
  @param geometry[in] A non-owning reference to the geometric representation of
  this collision element. One geometry can be shared among several collision
  elements.
  @param T_EG[in] Pose of the element's geometry in the frame of
  the element. In other words, a transformation from the geometry's frame into the
  collision element's frame. */
  CollisionElement(
      const DrakeShapes::Geometry* geometry,
      const Eigen::Isometry3d &T_EG);

  ~CollisionElement();

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
  // This allows for a single geometry resource to be shared among several
  // collision elements effectively reducing its footprint.
  const DrakeShapes::Geometry* geometry_{nullptr};

  // Geometry's frame to element's frame transformation. This is only used
  // before implementations are initialized as a temporary copy for when
  // implementations are instantiated.
  Eigen::Isometry3d T_EG_;
};

}  // end namespace collision
}  // end namespace drake
