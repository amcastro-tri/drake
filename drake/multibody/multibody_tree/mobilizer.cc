#include "drake/multibody/multibody_tree/joint.h"

#include "drake/common/drake_assert.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
void Joint<T>::set_parent_tree(MultibodyTree<T>* parent) {
  parent_tree_ = parent;
}

// Explicitly instantiates on the most common scalar types.
template class Joint<double>;

}  // namespace multibody
}  // namespace drake
