#include "drake/multibody/plant/discrete_update_manager.h"

#include <utility>

#include "drake/multibody/plant/multibody_plant_discrete_update_manager_attorney.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
const MultibodyTree<T>& DiscreteUpdateManager<T>::internal_tree() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::internal_tree(plant());
}

template <typename T>
void DiscreteUpdateManager<T>::AddInForcesFromInputPorts(
    const drake::systems::Context<T>& context, MultibodyForces<T>* forces) const{
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::AddInForcesFromInputPorts(
      plant(), context, forces);
}

template <typename T>
const internal::ContactJacobians<T>& DiscreteUpdateManager<
    T>::EvalContactJacobians(const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::EvalContactJacobians(
      plant(), context);
}

template <typename T>
std::vector<CoulombFriction<double>>
DiscreteUpdateManager<T>::CalcCombinedFrictionCoefficients(
    const drake::systems::Context<T>& context,
    const std::vector<internal::DiscreteContactPair<T>>& contact_pairs) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::CalcCombinedFrictionCoefficients(plant(), context, contact_pairs);
}

template <typename T>
std::vector<internal::DiscreteContactPair<T>>
DiscreteUpdateManager<T>::CalcDiscreteContactPairs(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::CalcDiscreteContactPairs(plant(), context);
}

template <typename T>
const contact_solvers::internal::ContactSolverResults<T>& DiscreteUpdateManager<
    T>::EvalContactSolverResults(const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::EvalContactSolverResults(plant(), context);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
