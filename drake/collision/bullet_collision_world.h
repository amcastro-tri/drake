#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/drakeCollisionEngine_export.h"
#include "drake/collision/bullet_collision_element.h"

namespace drake {
namespace collision {

/** Class representing a world of BulletCollisionElement's.
This class provides a Bullet implementation of a collision world to be used as
a back end for the handle class CollisionWorld.
Collision queries are handled by an internal Bullet implementation. */
class DRAKECOLLISIONENGINE_EXPORT BulletCollisionWorld {
 public:
  BulletCollisionWorld();
  ~BulletCollisionWorld();

  /** Adds CollisionElement @p e to this BulletCollisionWorld.
  @param[in] e the collision element to be added to this world.
  @returns A non-owning pointer to the recently added collision element. */
  BulletCollisionElement* add_collision_element(
      std::unique_ptr<BulletCollisionElement> e);

  /** Returns the number of collision elements in this collision world. */
  int get_num_elements() const;

  // This should take something like a ClosestPointsResult for output.
  void ClosestPointsAllToAll();

  // This should take something like a RayCastResult for output.
  void RayCast();
 private:
  // BulletCollisionWorld owns the Bullet collision elements.
  std::vector<std::unique_ptr<BulletCollisionElement>> collision_elements_;
};

}  // end namespace collision
}  // end namespace drake
