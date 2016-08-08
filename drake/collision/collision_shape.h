#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/drakeCollisionEngine_export.h"
#include "drake/systems/plants/shapes/DrakeShapes.h"

namespace drake {
namespace collision {

/** The geometric implementation for a collision element. */
// This is a handle class to its internal implementations (PIMPL's).
// Currently available implementations include: Bullet.
class DRAKECOLLISIONENGINE_EXPORT CollisionShape {
  // Grant the CollisionWorld access to this class' implementation.
  friend class CollisionWorld;

 public:
  virtual ~CollisionShape();

 protected:
  // The underlying back ends to CollisionShape.
  class BulletCollisionShape* bullet_pimpl_;

  // Only the CollisionWorld, which owns the collision shapes, has access to
  // this method.
  virtual std::unique_ptr<class BulletCollisionShape>
  InstantiateBulletImplementation() = 0;
};

/** A sphere collision shape. */
class DRAKECOLLISIONENGINE_EXPORT SphereCollisionShape: public CollisionShape {
 public:
  /** Constructs a CollisionSphere implementation for a sphere geometry. */
  SphereCollisionShape(const DrakeShapes::Sphere& sphere);

  ~SphereCollisionShape();

 private:
  double radius_;
  std::unique_ptr<class BulletCollisionShape>
  InstantiateBulletImplementation() override;
};

/** A box collision shape. */
class DRAKECOLLISIONENGINE_EXPORT BoxCollisionShape: public CollisionShape {
 public:
  /** Constructs a BulletCollisionBox of a given size. */
  BoxCollisionShape(const DrakeShapes::Box& box);

  ~BoxCollisionShape();

 private:
  Eigen::Vector3d size_;
  std::unique_ptr<class BulletCollisionShape>
  InstantiateBulletImplementation() override;
};


}  // end namespace collision
}  // end namespace drake
