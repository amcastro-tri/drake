#pragma once

// System headers.
#include <memory>
#include <vector>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/collision/collision_element.h"

namespace drake {
namespace collision {

// Forward declaration to implementation.
class CollisionWorldImpl;

class CollisionWorld {
 public:
  CollisionWorld();
  ~CollisionWorld();

  /** Adds CollisionElement @e to this CollisionWorld.
  @returns a non-owning pointer to the collision element just added. */
  CollisionElement* add_collision_element(std::unique_ptr<CollisionElement> e);

  DrakeShapes::Geometry* add_geometry(std::unique_ptr<DrakeShapes::Geometry> g);

  int get_num_elements() const;

  int get_num_geometries() const;

  void Initialize();

  // This should take something like a ClosestPointsResult for output.
  void ClosestPointsAllToAll();

  // This should take something like a RayCastResult for output.
  void RayCast();

 private:
  // The underlying back end implementation to CollisionWorld.
  std::unique_ptr<CollisionWorldImpl> pimpl_;

  // CollisionWorld owns the collision elements' handles.
  std::vector<std::unique_ptr<CollisionElement>> collision_elements_;
};

}  // end namespace collision
}  // end namespace drake
