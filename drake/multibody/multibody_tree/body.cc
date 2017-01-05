#include "drake/multibody/multibody_tree/body.h"

#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
Body<T>::Body(const MassProperties<double>& mass_properties) :
    parent_tree_(nullptr), default_mass_properties_(mass_properties) {}

template <typename T>
Body<T>::Body(MultibodyTree<T>* parent_tree,
              const MassProperties<double>& mass_properties) :
    parent_tree_(parent_tree), default_mass_properties_(mass_properties) {}

template <typename T>
Body<T>* Body<T>::CreateBody(
    MultibodyTree<T>* parent_tree,
    const MassProperties<double>& mass_properties) {
  DRAKE_ASSERT(parent_tree != nullptr);
  // We cannot use make_unique here since Body<T>'s constructor is private.
  return parent_tree->AddBody(
      std::unique_ptr<Body<T>>(new Body<T>(parent_tree, mass_properties)));
}

template <typename T>
void Body<T>::set_parent_tree(MultibodyTree<T>* parent) {
  parent_tree_ = parent;
}

// Explicitly instantiates on the most common scalar types.
template class Body<double>;

}  // namespace multibody
}  // namespace drake
