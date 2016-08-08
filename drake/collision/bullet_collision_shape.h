#pragma once

#include <memory>

#include "btBulletCollisionCommon.h"
#include <Eigen/Dense>

#include "drake/drakeCollisionEngine_export.h"
#include "drake/systems/plants/shapes/DrakeShapes.h"

namespace drake {
namespace collision {

/** The geometric implementation for a collision element. */
class DRAKECOLLISIONENGINE_EXPORT BulletCollisionShape {
 public:
  virtual ~BulletCollisionShape();

 protected:
  // The underlying Bullet implementation.
  std::unique_ptr<btCollisionShape> bt_shape_;
};

/** A sphere collision shape. */
class DRAKECOLLISIONENGINE_EXPORT BulletCollisionSphere:
    public BulletCollisionShape {
 public:
  /** Constructs a BulletCollisionSphere of a given radius. */
  BulletCollisionSphere(double radius);

  ~BulletCollisionSphere();
};

/** A box collision shape. */
class DRAKECOLLISIONENGINE_EXPORT BulletCollisionBox:
    public BulletCollisionShape {
 public:
  /** Constructs a BulletCollisionBox of a given size. */
  BulletCollisionBox(Eigen::Vector3d size);

  ~BulletCollisionBox();
};

}  // end namespace collision
}  // end namespace drake
