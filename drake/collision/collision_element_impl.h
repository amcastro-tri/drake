#pragma once

// System headers.
#include <memory>

// Third party headers.
#include <Eigen/Dense>

// Drake headers.
#include "drake/systems/plants/shapes/DrakeShapes.h"
#include "bullet_collision_element.h"

namespace drake {
namespace collision {

// Base (abstract) class for implementations.
class CollisionElementImpl: public CollisionElementInterface {
 public:
  CollisionElementImpl(
      const DrakeShapes::Geometry &geometry,
      const Eigen::Isometry3d &T_EG);

  void join_clique(int clique) override;

  void update_geometry_to_element_transform(
      const Eigen::Isometry3d &T_EG) override;

  virtual ~CollisionElementImpl() { }

 private:

  //friend class CollisionWorldImpl;
  // Called by CollisionWorldImpl::add_collision_element to set the underlying
  // pimpl to the Bullet back end implementation.
  //void set_bullet_pimpl(BulletCollisionElement* pimpl);

  // Non owned pointers to back end implementations.
  // The collision element implementation does not own its back ends
  // (thus raw pointers) but it is the corresponding collision world back ends
  // that own them.
  // For instance, BulletCollisionWorld owns its BulletCollisionElement's.
  BulletCollisionElement* bullet_pimpl_{nullptr};
  //std::unique_ptr<FCLCollisionElement> fcl_pimpl_;

  // Transform from geometry's frame into element's frame.
  // It is only used if non of the pimpl's were initialized.
  Eigen::Isometry3d T_EG_;
};

}  // end namespace collision
}  // end namespace drake
