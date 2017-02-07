#include "drake/multibody/multibody_tree/rigid_body.h"

#include "drake/common/drake_assert.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>

namespace drake {
namespace multibody {

template <typename T>
RigidBody<T>& RigidBody<T>::Create(MultibodyTree<T>* tree,
                                   const MassProperties<T>& mass_properties) {
  // Notice that here we cannot use std::make_unique since constructors are made
  // private to avoid users creating bodies by other means other than calling
  // Create().
  RigidBody<T>* body = new RigidBody<T>(mass_properties);
  // tree takes ownership.
  BodyIndex body_id = tree->AddBody(std::unique_ptr<Body<T>>(body));
  body->set_parent_tree(tree);
  body->set_id(body_id);

  // Create a BodyFrame associated with this body.
  BodyFrame<T>& body_frame = BodyFrame<T>::Create(tree, *body);
  body->set_body_frame(body_frame.get_id());

  return *body;
}


template <typename T>
RigidBody<T>::RigidBody(const MassProperties<double>& mass_properties) :
    default_mass_properties_(mass_properties) {}

#if 0
template <typename T>
RigidBodyFrame<T>& RigidBody<T>::RigidlyAttachFrame(const Isometry3<T>& X_BF) {
  auto owned_frame = std::make_unique<RigidBodyFrame<T>>(*this, X_BF);
  RigidBodyFrame<T>* frame =
      this->get_mutable_parent_tree()->AddMaterialFrame(std::move(owned_frame));
  material_frames_.push_back(frame->get_id());
  return *frame;
}
#endif

// Explicitly instantiates on the most common scalar types.
template class RigidBody<double>;

}  // namespace multibody
}  // namespace drake
