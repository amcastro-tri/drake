#pragma once

#include <memory>

#include "btBulletCollisionCommon.h"
#include <Eigen/Dense>

#include "drake/drakeCollisionEngine_export.h"
#include "drake/systems/plants/shapes/DrakeShapes.h"

namespace drake {
namespace collision {

/** A representation of the geometric shape of a collision element with
Bullet back end. */
class DRAKECOLLISIONENGINE_EXPORT BulletCollisionShape {
 public:
  virtual ~BulletCollisionShape();

  btCollisionShape* get_mutable_bullet_shape();

 protected:
  // The underlying Bullet implementation.
  std::unique_ptr<btCollisionShape> bt_shape_;
};

/** A sphere collision shape. */
class DRAKECOLLISIONENGINE_EXPORT BulletSphereShape:
    public BulletCollisionShape {
 public:
  /** Constructs a sphere centered at the origin with a given @p radius. */
  BulletSphereShape(double radius);

  ~BulletSphereShape();
};

/** A box collision shape. */
class DRAKECOLLISIONENGINE_EXPORT BulletBoxShape:
    public BulletCollisionShape {
 public:
  /** Constructs a box centered at the origin with sides of specified
  lengths in @p side_lengths. */
  BulletBoxShape(Eigen::Vector3d side_lengths);

  ~BulletBoxShape();
};

}  // end namespace collision
}  // end namespace drake
