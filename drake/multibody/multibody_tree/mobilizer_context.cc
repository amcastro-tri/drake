#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"

namespace drake {
namespace multibody {

template <typename T>
MobilizerPositionKinematics<T>::MobilizerPositionKinematics(
    const MultibodyTree<T>& tree,
    const MultibodyTreeContext<T>& context,
    MobilizerIndex mobilizer_id) {
    H_FM_ =
        tree.get_mutable_mobilizer_H_FM(context, mobilizer_id).mutable_data();
}

template <typename T>
MobilizerContext<T>::MobilizerContext(
    const MultibodyTree<T>& tree,
    const MultibodyTreeContext<T>& context,
    MobilizerIndex mobilizer_id) :
    position_kinematics_(tree, context, mobilizer_id)
{
  num_positions_ = tree.get_mobilizer(mobilizer_id).get_num_positions();
  num_velocities_ = tree.get_mobilizer(mobilizer_id).get_num_velocities();
  q_ = tree.get_mobilizer_positions(context, mobilizer_id).data();
}

// Explicitly instantiates on the most common scalar types.
template class MobilizerPositionKinematics<double>;

}  // namespace multibody
}  // namespace drake
