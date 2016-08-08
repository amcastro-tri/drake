#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/drakeCollisionEngine_export.h"
#include "drake/systems/plants/shapes/DrakeShapes.h"

namespace drake {
namespace collision {

/** A representation of the geometric shape of a collision element.
A collision shape can be shared between multiple collision elements. */
// This is a handle class to its internal implementations (PIMPL's).
// Currently available implementations include: BulletCollisionShape.
class DRAKECOLLISIONENGINE_EXPORT CollisionShape {
  // Collision shapes are manged by a CollisionWorld.
  // Here access to the implementation internals is granted to the
  // CollisionWorld.
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
  /** Constructs a sphere from a DrakeShapes::Sphere. */
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
  /** Constructs a box from a DrakeShapes::Box. */
  BoxCollisionShape(const DrakeShapes::Box& box);

  ~BoxCollisionShape();

 private:
  Eigen::Vector3d size_;
  std::unique_ptr<class BulletCollisionShape>
  InstantiateBulletImplementation() override;
};


}  // end namespace collision
}  // end namespace drake
