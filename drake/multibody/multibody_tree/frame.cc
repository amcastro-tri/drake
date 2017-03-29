#include "drake/multibody/multibody_tree/frame.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"

#include <memory>

namespace drake {
namespace multibody {

template <typename T>
MaterialFrame<T>::MaterialFrame(const Body<T>& body) {
  DRAKE_ASSERT(body.get_index().is_valid());
  topology_.body_id = body.get_index();
}

template <typename T>
BodyFrame<T>& BodyFrame<T>::Create(MultibodyTree<T>* tree, const Body<T>& body)
{
  // Notice that here we cannot use std::make_unique since constructors are made
  // private to avoid users creating frames by other means other than calling
  // Create().
  // Obtain the local id of this new frame in the body and add it into the body.
  BodyFrame<T>* frame = new BodyFrame<T>(body);
  FrameIndex frame_id =
      tree->AddMaterialFrame(std::unique_ptr<MaterialFrame<T>>(frame));
  frame->set_parent_tree(tree);
  frame->set_index(frame_id);
  return *frame;
}

template <typename T>
BodyFrame<T>::BodyFrame(
    const Body<T>& body) : MaterialFrame<T>(body) {}

template <typename T>
RigidBodyFrame<T>& RigidBodyFrame<T>::Create(
    MultibodyTree<T>* tree, const RigidBody<T>& body, const Isometry3<T>& X_BM)
{
  // Notice that here we cannot use std::make_unique since constructors are made
  // private to avoid users creating frames by other means other than calling
  // Create().
  RigidBodyFrame<T>* frame = new RigidBodyFrame<T>(body, X_BM);
  FrameIndex frame_id =
      tree->AddMaterialFrame(std::unique_ptr<MaterialFrame<T>>(frame));
  frame->set_parent_tree(tree);
  frame->set_index(frame_id);
  return *frame;
}

template <typename T>
RigidBodyFrame<T>::RigidBodyFrame(
    const RigidBody<T>& body, const Isometry3<T>& X_BM) :
    MaterialFrame<T>(body), X_BM_(X_BM) {}

template <typename T>
void RigidBodyFrame<T>::SetDefaults(MultibodyTreeContext<T>* context) {
  PositionKinematicsCache<T>* pc = context->get_mutable_position_kinematics();
  pc->get_mutable_X_BF(this->get_topology().X_BF_index) = X_BM_;
  pc->get_mutable_X_MB(this->get_topology().body_node) = X_BM_.inverse();
}

#if 0
template <typename T>
FixedOffsetFrame<T>::FixedOffsetFrame(
    const MaterialFrame<T>& M, const Isometry3<T>& X_MF) :
    MaterialFrame<T>(M.get_body_id()), X_MF_(X_MF),
    material_frame_id_(M.get_index()) {}
#endif

// Explicitly instantiates on the most common scalar types.
template class BodyFrame<double>;
template class RigidBodyFrame<double>;

}  // namespace multibody
}  // namespace drake
