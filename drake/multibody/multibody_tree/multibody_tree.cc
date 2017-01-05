#include "multibody_tree.h"

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

using Eigen::Vector3d;

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // The "world" body has infinite mass.
  Body<double>::CreateBody(this, MassProperties<double>::InfiniteMass());
}

template <typename T>
Body<T>* MultibodyTree<T>::AddBody(std::unique_ptr<Body<T>> body) {
  DRAKE_DEMAND(body != nullptr);
  // Call similar to invalidateSubsystemTopologyCache() in Simbody.
  // Notify that the topology of the MultibodyTree changed.
  InvalidateTopology();
  //body->set_id(get_num_bodies());
  Body<T>* body_ptr = body.get();
  body_ptr->set_parent_tree(this);
  bodies_.push_back(std::move(body));
  return body_ptr;
}

template class MultibodyTree<double>;

}  // namespace multibody
}  // namespace drake
