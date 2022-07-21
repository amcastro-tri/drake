#include "drake/multibody/plant/discrete_update_manager.h"

#include <utility>

#include "drake/multibody/plant/multibody_plant_discrete_update_manager_attorney.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
int DiscreteUpdateManager<T>::num_non_contact_constraints() const {
  // N.B. This must be updated whenever the manager's API supports adding other
  // constraint types.
  return coupler_constraints_sepcs_.size();
}

template <typename T>
void DiscreteUpdateManager<T>::AddCouplerConstraint(const Joint<T>& joint0,
                                                    const Joint<T>& joint1,
                                                    const T& gear_ratio) {
  DRAKE_THROW_UNLESS(joint0.num_velocities() == 1);
  DRAKE_THROW_UNLESS(joint1.num_velocities() == 1);
  coupler_constraints_sepcs_.push_back(CouplerConstraintSpecs{
      joint0.index(), joint1.index(), gear_ratio});
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<double>>
DiscreteUpdateManager<T>::CloneToDouble(const MultibodyPlant<double>*) const {
  throw std::logic_error(
      "Scalar conversion to double is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>>
DiscreteUpdateManager<T>::CloneToAutoDiffXd(const MultibodyPlant<AutoDiffXd>*) const {
  throw std::logic_error(
      "Scalar conversion to AutodiffXd is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>>
DiscreteUpdateManager<T>::CloneToSymbolic(const MultibodyPlant<symbolic::Expression>*) const {
  throw std::logic_error(
      "Scalar conversion to symbolic::Expression is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
bool DiscreteUpdateManager<T>::is_cloneable_to_double() const {
  return false;
}

template <typename T>
bool DiscreteUpdateManager<T>::is_cloneable_to_autodiff() const {
  return false;
}

template <typename T>
bool DiscreteUpdateManager<T>::is_cloneable_to_symbolic() const {
  return false;
}

template <typename T>
const MultibodyTree<T>& DiscreteUpdateManager<T>::internal_tree() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::internal_tree(plant());
}

template <typename T>
systems::CacheEntry& DiscreteUpdateManager<T>::DeclareCacheEntry(
    std::string description, systems::ValueProducer value_producer,
    std::set<systems::DependencyTicket> prerequisites_of_calc,
    MultibodyPlant<T>* plant) {
  DRAKE_DEMAND(plant != nullptr);
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::DeclareCacheEntry(
      plant, std::move(description), std::move(value_producer),
      std::move(prerequisites_of_calc));
}

template <typename T>
const contact_solvers::internal::ContactSolverResults<T>&
DiscreteUpdateManager<T>::EvalContactSolverResults(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::EvalContactSolverResults(plant(), context);
}

template <typename T>
const internal::ContactJacobians<T>&
DiscreteUpdateManager<T>::EvalContactJacobians(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::EvalContactJacobians(
      plant(), context);
}

template <typename T>
const std::vector<internal::DiscreteContactPair<T>>&
DiscreteUpdateManager<T>::EvalDiscreteContactPairs(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::EvalDiscreteContactPairs(plant(), context);
}

template <typename T>
const std::vector<geometry::ContactSurface<T>>&
DiscreteUpdateManager<T>::EvalContactSurfaces(
    const systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::EvalContactSurfaces(
      plant(), context);
}

template <typename T>
std::vector<CoulombFriction<double>>
DiscreteUpdateManager<T>::CalcCombinedFrictionCoefficients(
    const systems::Context<T>& context,
    const std::vector<internal::DiscreteContactPair<T>>& contact_pairs) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::CalcCombinedFrictionCoefficients(plant(), context, contact_pairs);
}

template <typename T>
void DiscreteUpdateManager<T>::AddInForcesFromInputPorts(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::AddInForcesFromInputPorts(
      plant(), context, forces);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcNonContactForces(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::CalcNonContactForces(
      plant(), context, forces);
}

template <typename T>
ScopeExit DiscreteUpdateManager<T>::ThrowIfNonContactForceInProgress(
    const drake::systems::Context<T>& context) const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::ThrowIfNonContactForceInProgress(plant(), context);
}

template <typename T>
void DiscreteUpdateManager<T>::CalcForceElementsContribution(
    const drake::systems::Context<T>& context,
    MultibodyForces<T>* forces) const {
  MultibodyPlantDiscreteUpdateManagerAttorney<T>::CalcForceElementsContribution(
      plant(), context, forces);
}

template <typename T>
const std::vector<std::vector<geometry::GeometryId>>&
DiscreteUpdateManager<T>::collision_geometries() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::collision_geometries(
      plant());
}

template <typename T>
double DiscreteUpdateManager<T>::default_contact_stiffness() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::default_contact_stiffness(plant());
}

template <typename T>
double DiscreteUpdateManager<T>::default_contact_dissipation() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::default_contact_dissipation(plant());
}

template <typename T>
const std::unordered_map<geometry::GeometryId, BodyIndex>&
DiscreteUpdateManager<T>::geometry_id_to_body_index() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::geometry_id_to_body_index(*plant_);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
