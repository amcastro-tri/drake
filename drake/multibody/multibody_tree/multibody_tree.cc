#include "multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
Body<T>* MultibodyTree<T>::AddBody(std::unique_ptr<Body<T>> body) {
  DRAKE_DEMAND(body != nullptr);
  // Call similar to invalidateSubsystemTopologyCache() in Simbody.
  // Notify that the topology of the MultibodyTree changed.
  InvalidateTopology();
  //body->set_id(get_num_bodies());
  Body<T>* body_ptr = body.get();
  bodies_.push_back(std::move(body));
  return body_ptr;
}

template class MultibodyTree<double>;

}  // namespace multibody
}  // namespace drake
