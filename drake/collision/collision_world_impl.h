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
  CollisionWorldImpl();
  virtual ~CollisionWorldImpl();

  CollisionElementImpl* add_collision_element(CollisionElementImpl* e);

  DrakeShapes::Geometry* add_geometry(std::unique_ptr<DrakeShapes::Geometry> g);

  int get_num_elements() const override;

  int get_num_geometries() const override;

  void Initialize() override;

  // This should take something like a ClosestPointsResult for output.
  void ClosestPointsAllToAll() override;

  // This should take something like a RayCastResult for output.
  void RayCast() override;
 private:
  // Back end implementations
  std::unique_ptr<BulletCollisionWorld> bullet_pimpl_;
  //std::unique_ptr<FCLCollisionWorld> fcl_pimpl_;

  // Non-owned references to collision element implementations.
  // CollisionElementImpl's are owned by their CollisionElement handles which in
  // turn are owned by CollisionWorld.
  std::vector<CollisionElementImpl*> collision_elements_;

  std::vector<std::unique_ptr<DrakeShapes::Geometry>> geometries_;
};

}  // end namespace collision
}  // end namespace drake
