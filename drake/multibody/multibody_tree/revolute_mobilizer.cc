#include "drake/multibody/multibody_tree/revolute_mobilizer.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace multibody {

template <typename T>
const T& RevoluteMobilizer<T>::get_angle(
    const systems::Context<T>& context) const {
  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);
  auto q = this->get_positions(mbt_context);
  DRAKE_ASSERT(q.size() == nq);
  return q.coeffRef(0);
}

template <typename T>
const RevoluteMobilizer<T>& RevoluteMobilizer<T>::set_angle(
    systems::Context<T>* context, const T& angle) const {
  auto mbt_context = dynamic_cast<MultibodyTreeContext<T>*>(context);
  DRAKE_DEMAND(mbt_context != nullptr);
  auto q = this->get_mutable_positions(mbt_context);
  DRAKE_ASSERT(q.size() == nq);
  q[0] = angle;
  return *this;
}

template <typename T>
void RevoluteMobilizer<T>::CalcAcrossMobilizerTransform(
    const MultibodyTreeContext<T>& context,
    PositionKinematicsCache<T>* pc) const {
  auto q = this->get_positions(context);
  Isometry3<T>& X_FM = this->get_mutable_X_FM(pc);
  X_FM = Isometry3<T>::Identity();
  X_FM.linear() = Eigen::AngleAxis<T>(q[0], axis_F_).toRotationMatrix();
}

// Explicitly instantiates on the most common scalar types.
template class RevoluteMobilizer<double>;
template class RevoluteMobilizer<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
