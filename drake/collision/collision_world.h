#pragma once

// System headers.
#include <memory>
#include <vector>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/drakeCollisionEngine_export.h"
#include "drake/collision/collision_element.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

class DRAKECOLLISIONENGINE_EXPORT CollisionWorld {
 public:
  CollisionWorld();
  ~CollisionWorld();

  /** Adds CollisionElement @e to this CollisionWorld.
  @param[in] e the collision element to be added to this world.
  @returns a non-owning pointer to the recently added collision element. */
  CollisionElement* add_collision_element(std::unique_ptr<CollisionElement> e);

  /** Adds DrakeShape::Geometry @p g to be managed by this collision world.
  @param g the geometry to be managed by this world.
  @returns a non-owning pointer to the recently added geometry. */
  template <class DerivedGeometry>
  DerivedGeometry* add_geometry(std::unique_ptr<DerivedGeometry> g) {
    PRINT_VAR(__PRETTY_FUNCTION__);
    DerivedGeometry* gptr = g.get();
    collision_shapes_.push_back(move(g));
    return gptr;
  }

  /** Returns the number of collision elements in this collision world. */
  int get_num_elements() const;

  /** Returns the number of geometries managed by this collision world.
  The number of geometries does not neccessarily need to match the number of
  collision elements as reported by get_num_elements() since a single geometry
  can be shared by several collision elements. */
  int get_num_geometries() const;

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

  void InitializeBulletCollisionElements();

  // The underlying back end implementations to CollisionWorld.
  // These pointers are managed by CollisionWorld, the handle, and therefore
  // properly deleted in its destructor.
  class BulletCollisionWorld* bullet_pimpl_{nullptr};

  // CollisionWorld owns the collision elements' handles.
  std::vector<std::unique_ptr<CollisionElement>> collision_elements_;
  std::vector<std::unique_ptr<DrakeShapes::Geometry>> collision_shapes_;
};

}  // end namespace collision
}  // end namespace drake
