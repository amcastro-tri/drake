#include "drake/multibody/tree/custom_force_element.h"

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
CustomForceElement<T>::CustomForceElement(ModelInstanceIndex model_instance)
    : ForceElement<T>(model_instance) {}

template <typename T>
T CustomForceElement<T>::CalcPotentialEnergy(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>&) const {
  return CalcPotentialEnergy(context);
}

template <typename T>
T CustomForceElement<T>::CalcConservativePower(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&) const {
  return CalcConservativePower(context);
}

template <typename T>
T CustomForceElement<T>::CalcNonConservativePower(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&) const {
  return CalcNonConservativePower(context);
}

template <typename T>
std::unique_ptr<ForceElement<double>> CustomForceElement<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>&) const {
  return ToDouble();
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>> CustomForceElement<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>&) const {
  return ToAutoDiffXd();
}

template <typename T>
std::unique_ptr<ForceElement<symbolic::Expression>>
CustomForceElement<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>&) const {
  return ToSymbolic();
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::CustomForceElement)
