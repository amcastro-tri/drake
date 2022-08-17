#include "drake/multibody/plant/discrete_update_manager.h"

#include <utility>

#include "drake/multibody/plant/multibody_plant_discrete_update_manager_attorney.h"

namespace drake {
namespace multibody {
namespace internal {

using contact_solvers::internal::ContactSolverResults;

template <typename T>
void DiscreteUpdateManager<T>::DeclareCacheEntries() {
  // Cache discrete contact pairs.
  const auto& contact_solver_results_cache_entry = this->DeclareCacheEntry(
      "Discrete update results.",
      systems::ValueProducer(
          this, &DiscreteUpdateManager<T>::CalcContactSolverResults),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.contact_solver_results =
      contact_solver_results_cache_entry.cache_index();

  // Cache discrete contact pairs.
  const auto& contact_results_cache_entry = this->DeclareCacheEntry(
      "Contact results.",
      systems::ValueProducer(this,
                             &DiscreteUpdateManager<T>::CalcContactResults),
      {systems::System<T>::xd_ticket(),
       systems::System<T>::all_parameters_ticket()});
  cache_indexes_.contact_results = contact_results_cache_entry.cache_index();

  // Accelerations consistent with the discrete update.
  const auto& acceleration_kinematics_cache_entry = this->DeclareCacheEntry(
      "Acceleration kinematics.",
      systems::ValueProducer(
          this, &DiscreteUpdateManager<T>::CalcAccelerationKinematicsCache),
      {plant().cache_entry_ticket(cache_indexes_.contact_solver_results)});
  cache_indexes_.acceleration_kinematics =
      acceleration_kinematics_cache_entry.cache_index();

  // Now allow concrete instances to declare the cache entries they need.
  DoDeclareCacheEntries();
}

template <typename T>
const ContactSolverResults<T>&
DiscreteUpdateManager<T>::EvalContactSolverResults(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_solver_results)
      .template Eval<ContactSolverResults<T>>(context);
}

template <typename T>
const ContactResults<T>& DiscreteUpdateManager<T>::EvalContactResults(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.contact_results)
      .template Eval<ContactResults<T>>(context);
}

template <typename T>
const internal::AccelerationKinematicsCache<T>&
DiscreteUpdateManager<T>::EvalAccelerationKinematicsCache(
    const systems::Context<T>& context) const {
  return plant()
      .get_cache_entry(cache_indexes_.acceleration_kinematics)
      .template Eval<internal::AccelerationKinematicsCache<T>>(context);
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<double>>
DiscreteUpdateManager<T>::CloneToDouble() const {
  throw std::logic_error(
      "Scalar conversion to double is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<AutoDiffXd>>
DiscreteUpdateManager<T>::CloneToAutoDiffXd() const {
  throw std::logic_error(
      "Scalar conversion to AutodiffXd is not supported by this "
      "DiscreteUpdateManager.");
}

template <typename T>
std::unique_ptr<DiscreteUpdateManager<symbolic::Expression>>
DiscreteUpdateManager<T>::CloneToSymbolic() const {
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
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
  DRAKE_DEMAND(mutable_plant_ != nullptr);
  DRAKE_DEMAND(mutable_plant_ == plant_);
  return MultibodyPlantDiscreteUpdateManagerAttorney<T>::DeclareCacheEntry(
      mutable_plant_, std::move(description), std::move(value_producer),
      std::move(prerequisites_of_calc));
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

template <typename T>
const std::vector<internal::CouplerConstraintSpecs<T>>&
DiscreteUpdateManager<T>::coupler_constraints_specs() const {
  return MultibodyPlantDiscreteUpdateManagerAttorney<
      T>::coupler_constraints_specs(*plant_);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DiscreteUpdateManager);
