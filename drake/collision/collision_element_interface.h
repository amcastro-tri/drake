#pragma once

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.

namespace drake {
namespace collision {

class CollisionElementInterface {
 public:
  virtual ~CollisionElementInterface() {}

  virtual void join_clique(int clique) = 0;

  virtual void update_geometry_to_element_transform(
      const Eigen::Isometry3d &T_EG) = 0;
};

}  // end namespace collision
}  // end namespace drake
