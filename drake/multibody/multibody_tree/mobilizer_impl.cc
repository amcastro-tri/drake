#include "drake/multibody/multibody_tree/mobilizer_impl.h"

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

namespace drake {
namespace multibody {

template <typename T, int  nq, int nv>
void MobilizerImpl<T, nq, nv>::UpdatePositionKinematicsCache(
    const MultibodyTreeContext<T>& context) const {
  PositionKinematicsCache<T>* pc = context.get_mutable_position_kinematics();
  this->CalcAcrossMobilizerTransform(context, pc);
  this->CalcAcrossMobilizerVelocityJacobian(context, pc);
}

// Macro used to explicitly instantiate implementations on all sizes needed.
#define EXPLICITLY_INSTANTIATE(T) \
template class MobilizerImpl<T, 1, 1>;

// Explicitly instantiates on the most common scalar types.
EXPLICITLY_INSTANTIATE(double);

}  // namespace multibody
}  // namespace drake
