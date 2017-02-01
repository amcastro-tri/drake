#include "drake/multibody/multibody_tree/frame.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
BodyFrame<T>::BodyFrame(const Body<T>& body) : body_id_(body.get_id()) {}

template <typename T>
BodyFrame<T>::BodyFrame(BodyIndex body_id) : body_id_(body_id) {}

template <typename T>
const Body<T>& BodyFrame<T>::get_body() const {
  return this->get_parent_tree().get_body(body_id_);
}

template <typename T>
BodyIndex BodyFrame<T>::get_body_id() const {
  return body_id_;
}

template <typename T>
RigidBodyFrame<T>::RigidBodyFrame(
    const Body<T>& body, const Isometry3<T>& X_BF) :
    BodyFrame<T>(body), X_BF_(X_BF) {}

// Explicitly instantiates on the most common scalar types.
template class BodyFrame<double>;
template class RigidBodyFrame<double>;

}  // namespace multibody
}  // namespace drake
