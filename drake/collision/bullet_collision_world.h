#pragma once

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/collision/collision_world_interface.h"
#include "drake/collision/bullet_collision_element.h"

namespace drake {
namespace collision {

class BulletCollisionWorld: public CollisionWorldInterface {
 public:
  BulletCollisionWorld();
  ~BulletCollisionWorld();

  /** Adds BulletCollisionElement @e to this BulletCollisionWorld.
  @returns a non-owning pointer to the collision element just added. */
  BulletCollisionElement* add_collision_element(
      std::unique_ptr<BulletCollisionElement> e);

  int get_number_of_elements() const override;

  int get_number_of_geometries() const override;

  void Initialize() override;

  // This should take something like a ClosestPointsResult for output.
  void ClosestPointsAllToAll() override;

  // This should take something like a RayCastResult for output.
  void RayCast() override;
 private:
  // BulletCollisionWorld owns the Bullet collision elements.
  std::vector<std::unique_ptr<BulletCollisionElement>> collision_elements_;
};

}  // end namespace collision
}  // end namespace drake
