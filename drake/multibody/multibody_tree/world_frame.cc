#include "drake/multibody/multibody_tree/world_frame.h"

#include <memory>
#include <stdexcept>

#include "drake/common/autodiff.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
template <typename ToScalar>
std::unique_ptr<Frame<ToScalar>> WorldFrame<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const BodyFrame<ToScalar>& body_frame_clone =
      tree_clone.get_variant(this->get_implementation());
  return std::make_unique<WorldFrame<ToScalar>>(body_frame_clone);
}

template <typename T>
std::unique_ptr<Frame<double>> WorldFrame<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<AutoDiffXd>> WorldFrame<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class WorldFrame<double>;
template class WorldFrame<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
