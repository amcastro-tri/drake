#include "drake/multibody/multibody_tree/link.h"

#include <memory>
#include <stdexcept>

#include "drake/common/autodiff.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
Link<T>::Link(const std::string& name, const SpatialInertia<double> M_BBo_B) :
    name_(name), default_spatial_inertia_(M_BBo_B), link_frame_(*this) {}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Link<ToScalar>>
Link<T>::TemplatedDoCloneToScalar(const MultibodyTree<ToScalar>&) const {
  // Make the Joint<T> clone.
  auto link_clone = std::make_unique<Link<ToScalar>>(
      this->get_name(), this->get_default_spatial_inertia());
  return std::move(link_clone);
}

template <typename T>
std::unique_ptr<Link<double>> Link<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Link<AutoDiffXd>> Link<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Frame<ToScalar>> LinkFrame<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Link<ToScalar>& link_clone = tree_clone.get_variant(this->get_link());
  // LinkFrame's constructor cannot be called from std::make_unique since it is
  // private and therefore we use "new".
  return std::unique_ptr<LinkFrame<ToScalar>>(
      new LinkFrame<ToScalar>(link_clone));
}

template <typename T>
std::unique_ptr<Frame<double>> LinkFrame<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<AutoDiffXd>> LinkFrame<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class LinkFrame<double>;
template class LinkFrame<AutoDiffXd>;

template class Link<double>;
template class Link<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
