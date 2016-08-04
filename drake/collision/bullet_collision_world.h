#pragma once

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/collision/bullet_collision_element.h"

namespace drake {
namespace collision {

class BulletCollisionWorld {
 public:
  BulletCollisionWorld();
  ~BulletCollisionWorld();

  /** Adds BulletCollisionElement @e to this BulletCollisionWorld.
  @returns a non-owning pointer to the collision element just added. */
  BulletCollisionElement* add_collision_element(
      std::unique_ptr<BulletCollisionElement> e);

  int get_num_elements() const;

  int get_num_geometries() const;

  void Initialize() ;

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
