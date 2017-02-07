#include "drake/multibody/multibody_tree/rigid_body.h"

#include "drake/common/drake_assert.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
FixedOffsetFrame<T>& SoftBody<T>::AttachFixedOffsetFrame(
    const SoftBodyFrame<T>& M, const Isometry3<T>& X_MF) {
  // First checks both, this SoftBody and the SoftBodyFrame belong to a
  // MultibodyTree and that it is the same.
  this->HasSameParentTreeOrThrows(M);

  // Checks if frame M actually belongs to this body.
  if (std::find(soft_body_frames_.begin(), soft_body_frames_.end(),
                M.get_id() == soft_body_frames_.end())) {
    throw std::runtime_error(
        "SoftBodyFrame M does not belong to this SoftBody.");
  }

  // Create a new FixedOffsetFrame attached to material frame M.
  FixedOffsetFrame<T>* frame =
      this->get_mutable_parent_tree()->AddMaterialFrame(
          std::make_unique<FixedOffsetFrame<T>>(M, X_MF));

  // Save the pair (soft_body_frame_id, fixed_offset_frame_id).
  fixed_offset_frames_.push_back({M.get_id(), frame->get_id()});
  return *frame;
}

// Explicitly instantiates on the most common scalar types.
template class RigidBody<double>;

}  // namespace multibody
}  // namespace drake
