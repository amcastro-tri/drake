#include "drake/multibody/multibody_tree/body.h"

#include "drake/common/drake_assert.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
Body<T>::Body(const MassProperties<double>& mass_properties) :
    default_mass_properties_(mass_properties) {}

template <typename T>
Body<T>* Body<T>::CreateBody(
    MultibodyTree<T>* parent_tree,
    const MassProperties<double>& mass_properties) {
  DRAKE_ASSERT(parent_tree != nullptr);
  auto body = std::make_unique<Body<T>>(mass_properties);
  body->set_parent_tree(parent_tree);
  return parent_tree->AddBody(std::move(body));
}

template <typename T>
void Body<T>::set_parent_tree(MultibodyTree<T>* parent) {
  parent_tree_ = parent;
}

template <typename T>
const Body<T>& Body<T>::get_inboard_body() const {
  return parent_tree_->get_body_inboard_body(get_id());
  // or ... ?
  // return parent_tree_->get_body(topology_.parent_body_);
}

template <typename T>
BodyIndex Body<T>::get_id() const {
  return topology_.id;
}

// Explicitly instantiates on the most common scalar types.
template class Body<double>;

}  // namespace multibody
}  // namespace drake
