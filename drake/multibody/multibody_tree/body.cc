#include "drake/multibody/multibody_tree/body.h"

#include "drake/common/drake_assert.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
Body<T>::Body(const MassProperties<double>& mass_properties) :
    default_mass_properties_(mass_properties) {}

template <typename T>
RigidBodyFrame<T>& Body<T>::RigidlyAttachFrame(const Isometry3<T>& X_BF) {
  RigidBodyFrame<T>* frame =  this->get_mutable_parent_tree()->AddFrame(
      std::make_unique<RigidBodyFrame<T>>(*this, X_BF));
  frames_.push_back(frame->get_id());
  return *frame;
}

// Explicitly instantiates on the most common scalar types.
template class Body<double>;

}  // namespace multibody
}  // namespace drake
