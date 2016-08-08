#pragma once

#include <memory>
#include <vector>

#include "drake/drakeCollisionEngine_export.h"
#include "drake/collision/collision_element.h"
#include "drake/collision/collision_shape.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

/** Class representing the world to which CollisionElement's belong.
It provides the interface to perform a wide set of collision queries including
naive O(n^2) all to all distance computations, narrow/broad phase collision
queries and ray casting. */
class DRAKECOLLISIONENGINE_EXPORT CollisionWorld {
 public:
  CollisionWorld();
  ~CollisionWorld();

  /** Adds CollisionElement @p e to this CollisionWorld.
  @param[in] e the collision element to be added to this world.
  @returns A non-owning pointer to the recently added collision element. */
  CollisionElement* add_collision_element(std::unique_ptr<CollisionElement> e);

  /** Adds a sphere collision element. */
  CollisionElement* add_collision_element(const DrakeShapes::Sphere& sphere);

  /** Adds a CollisionShape @p shape to be managed by this collision world.
  @param shape the collision shape to be managed by this world.
  @returns A non-owning pointer to the recently added shape. */
  CollisionShape* add_collision_shape(std::unique_ptr<CollisionShape> shape);

  /** Returns the number of collision elements in this collision world. */
  int get_num_elements() const;

  /** Returns the number of shapes managed by this collision world.
  The number of shapes does not neccessarily need to match the number of
  collision elements as reported by get_num_elements() since a single shape
  can be shared by several collision elements. */
  int get_num_shapes() const;

  /** Initializes the collision world to be ready for collision queries. After
  this call the user cannot add any more collision elements. */
  void Initialize();

  // This should take something like a ClosestPointsResult for output.
  // It should also take something like a ClosestPointQuery, FCL uses that
  // model.
  // See fcl/test/test_fcl_capsule_box_1.cpp
  void ClosestPointsAllToAll();

  // This should take something like a RayCastResult for output.
  void RayCast();

 private:

  // Instantiates the underlying Bullet implementation so that this collision
  // world is ready to handle queries managed by Bullet.
  void InstantiateBulletImplementation();

  // The underlying back end implementations to CollisionWorld.
  std::unique_ptr<class BulletCollisionWorld> bullet_pimpl_;

  // CollisionWorld owns the collision elements' handles.
  std::vector<std::unique_ptr<CollisionElement>> collision_elements_;
  std::vector<std::unique_ptr<CollisionShape>> collision_shapes_;
};

}  // end namespace collision
}  // end namespace drake
