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
  Body<T>* body_ptr = body.get();
  Body<T>::TopologyAccess::set_parent_tree(body_ptr, this);
  Body<T>::TopologyAccess::set_id(body_ptr, BodyIndex(get_num_bodies()));
  bodies_.push_back(std::move(body));
  return body_ptr;
}

template <typename T>
void MultibodyTree<T>::CompileTopology() {
  for (BodyIndex ibody(0); ibody < get_num_bodies(); ++ibody) {
    BodyTopology body_topology;
   // Body<T>::TopologyAccess::set_topology(
     //   &get_mutable_body(ibody), body_topology);
  }
}

template <typename T>
void MultibodyTree<T>::Compile() {
  // Fill in MultibodyTree<T>::body_levels_
  // It is assumed there is one joint per body.
  DRAKE_ASSERT(get_num_bodies() == get_num_joints());

  CompileTopology();
}

template <typename T>
const Body<T>& MultibodyTree<T>::get_body(BodyIndex body_id) const {
  DRAKE_ASSERT(body_id.is_valid() && body_id < get_num_bodies());
  return *bodies_[body_id];
}

template <typename T>
Body<T>& MultibodyTree<T>::get_mutable_body(BodyIndex body_id) const {
  DRAKE_ASSERT(body_id.is_valid() && body_id < get_num_bodies());
  return *bodies_[body_id];
}


template <typename T>
const Body<T>& MultibodyTree<T>::get_body_inboard_body(
    BodyIndex body_id) const {
  return get_body(get_body(body_id).get_topology().parent_body_);
}

template <typename T>
void MultibodyTree<T>::UpdatePositionKinematics(PositionKinematicsCache<T>* pc) const {

}

template class MultibodyTree<double>;

}  // namespace multibody
}  // namespace drake
