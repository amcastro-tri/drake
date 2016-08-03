#pragma once

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.

namespace drake {
namespace collision {

class CollisionWorldInterface {
 public:
  virtual ~CollisionWorldInterface() {}

  int get_number_of_elements() const = 0;

  virtual void Initialize() = 0;

  // This should take something like a ClosestPointsResult for output.
  virtual void ClosestPointsAllToAll() = 0;

  // This should take something like a RayCastResult for output.
  virtual void RayCast() = 0;
};

}  // end namespace collision
}  // end namespace drake
