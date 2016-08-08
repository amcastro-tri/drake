#pragma once

#include <memory>

#include <Eigen/Dense>

#include "btBulletCollisionCommon.h"

#include "drake/drakeCollisionEngine_export.h"
#include "drake/collision/bullet_collision_shape.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

/** An element of a BulletCollisionWorld representing a geometry with a given
pose the world. The internal implementation uses Bullet as a back end. */
// It is a possible implementation for the handle class CollisionElement.
class DRAKECOLLISIONENGINE_EXPORT BulletCollisionElement {
public:
  /**
  @param geometry[in] A non-owning reference to the geometric representation of
  this collision element. One geometry can be shared among several collision
  elements.
  @param T_EG[in] Pose of the element's geometry in the frame of
  the element. In other words, a transformation from the geometry's frame into the
  collision element's frame. */
  BulletCollisionElement(
      const BulletCollisionShape* geometry,
      const Eigen::Isometry3d& T_EG);

  ~BulletCollisionElement();

  /** Sets the transformation from the underlying geomety's frame to this
  collision element's frame. */
  void set_geometry_to_element_transform(
      const Eigen::Isometry3d &T_EG);

private:
  std::unique_ptr<btCollisionObject> bt_collision_object_;
};

}  // end namespace collision
}  // end namespace drake
