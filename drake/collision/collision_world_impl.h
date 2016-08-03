#pragma once

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/collision/bullet_collision_world.h"
#include "drake/collision/collision_element_impl.h"

namespace drake {
namespace collision {

class CollisionWorldImpl: public CollisionWorldInterface {
 public:
  virtual ~CollisionWorldImpl();

  CollisionElement* add_collision_element(
      std::unique_ptr<CollisionElementImpl> e);

  void Initialize() override;

  // This should take something like a ClosestPointsResult for output.
  void ClosestPointsAllToAll() override;

  // This should take something like a RayCastResult for output.
  void RayCast() override;
 private:
  // Back end implementations
  std::unique_ptr<BulletCollisionWorld> bullet_pimpl_;
  //std::unique_ptr<FCLCollisionWorld> fcl_pimpl_;
};

}  // end namespace collision
}  // end namespace drake
