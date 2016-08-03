#pragma once

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/systems/plants/shapes/DrakeShapes.h"

namespace drake {
namespace collision {

class CollisionElementInterface {
 public:
  virtual ~CollisionElementInterface() {}

  virtual void update_geometry_to_element_transform(
      const Eigen::Isometry3d &T_EG) = 0;
};

}  // end namespace collision
}  // end namespace drake
