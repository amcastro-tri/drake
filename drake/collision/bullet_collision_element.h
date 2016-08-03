#pragma once

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/systems/plants/shapes/DrakeShapes.h"
#include "drake/collision/collision_element_interface.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

// A particular implementation for Bullet collision elements
// This will live in its own header + cc files with the appropriate Bullet includes.
class BulletCollisionElement: public CollisionElementInterface {
public:
  /**
  @param geometry[in] The geometric model of the collision element.
  @param T_EG[in] pose of the element's geometry in the frame of
  the element. In other words, a transformation from the geometry's frame into the
  collision element's frame.**/
  BulletCollisionElement(
      const DrakeShapes::Geometry &geometry,
      const Eigen::Isometry3d &T_EG);

  ~BulletCollisionElement();

  void join_clique(int clique);

  void update_geometry_to_element_transform(
      const Eigen::Isometry3d &T_EG);

private:
  //std::unique_ptr<btCollisionObject> bt_obj_;
};

}  // end namespace collision
}  // end namespace drake
