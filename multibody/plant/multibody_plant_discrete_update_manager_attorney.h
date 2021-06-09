#pragma once
#include <set>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {
template <typename T>
class DiscreteUpdateManager;

/* This class is used to grant access to a selected collection of
 MultibodyPlant's private methods to DiscreteUpdateManager.

 @tparam_default_scalar */
template <typename T>
class MultibodyPlantDiscreteUpdateManagerAttorney {
 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantDiscreteUpdateManagerAttorney);

  friend class DiscreteUpdateManager<T>;

  static const MultibodyTree<T>& internal_tree(const MultibodyPlant<T>& plant) {
    return plant.internal_tree();
  }

  static void AddInForcesFromInputPorts(
      const MultibodyPlant<T>& plant, const drake::systems::Context<T>& context,
      MultibodyForces<T>* forces) {
    plant.AddInForcesFromInputPorts(context, forces);
  }

  static const internal::ContactJacobians<T>& EvalContactJacobians(
      const MultibodyPlant<T>& plant, const systems::Context<T>& context) {
    return plant.EvalContactJacobians(context);
  }

  static std::vector<CoulombFriction<double>> CalcCombinedFrictionCoefficients(
    const MultibodyPlant<T>& plant, 
      const drake::systems::Context<T>& context,
      const std::vector<internal::DiscreteContactPair<T>>& contact_pairs) {
    return plant.CalcCombinedFrictionCoefficients(context, contact_pairs);
  }

  static std::vector<internal::DiscreteContactPair<T>> CalcDiscreteContactPairs(
    const MultibodyPlant<T>& plant, 
      const systems::Context<T>& context) {
    return plant.CalcDiscreteContactPairs(context);
  }

  static const contact_solvers::internal::ContactSolverResults<T>&
  EvalContactSolverResults(const MultibodyPlant<T>& plant,
                           const systems::Context<T>& context) {
    return plant.EvalContactSolverResults(context);
  }

};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
