#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// CompliantContactManager computes the contact Jacobian J_AcBc_C for the
// relative velocity at a contact point Co between two geometries A and B,
// expressed in a contact frame C with Cz coincident with the contact normal.
// This structure is used to cache J_AcBc_C and rotation R_WC.
template <typename T>
struct ContactJacobianCache {
  // Contact Jacobian J_AcBc_C. Jc.block(3*i, 0, 3, nv), where nv is the number
  // of velocities in the model, corresponds to J_AcBc_C for the i-th contact
  // pair.
  MatrixX<T> Jc;

  // Rotation matrix to re-express between contact frame C and world frame W.
  // R_WC_list[i] corresponds to rotation R_WC for the i-th contact pair.
  std::vector<drake::math::RotationMatrix<T>> R_WC_list;
};

// This class implements the interface given by DiscreteUpdateManager so that
// contact computations can be consumed by MultibodyPlant.
//
// In particular, this manager sets up a contact problem where each of the
// bodies in the MultibodyPlant model are compliant, i.e. the contact model does
// not introduce state. Supported models include point contact with a linear
// model of compliance, see GetPointContactStiffness() and the Hydroelastic
// contact model, see @ref mbp_hydroelastic_materials_properties in
// MultibodyPlant's Doxygen documentation.
// Dissipation is modeled using a linear model. For point contact, given the
// penetration distance x and its time derivative ẋ, the normal contact force
// (in Newtons) is modeled as:
//   fₙ = k⋅(x + τ⋅ẋ)₊
// where k is the point contact stiffness, see GetPointContactStiffness(), τ is
// the dissipation time scale, and ()₊ corresponds to the "positive part"
// operator.
// Similarly, for hydroelastic contact the normal traction p (in Pascals) is:
//   p = (p₀+τ⋅dp₀/dn⋅ẋ)₊
// where p₀ is the object-centric virtual pressure field introduced by the
// hydroelastic model.
//
// @tparam_nonsymbolic_scalar
template <typename T>
class CompliantContactManager : public internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CompliantContactManager)

  // Constructs a contact manager that takes ownership of the supplied
  // `contact_solver` to solve the underlying contact problem.
  // @pre contact_solver != nullptr.
  CompliantContactManager(
      std::unique_ptr<contact_solvers::internal::ContactSolver<T>>
          contact_solver);

  ~CompliantContactManager() = default;

 private:
  // Struct used to conglomerate the indexes of cache entries declared by the
  // manager.
  struct CacheIndexes {
    systems::CacheIndex discrete_contact_pairs;
    systems::CacheIndex contact_jacobian;
  };

  using internal::DiscreteUpdateManager<T>::plant;

  // Provide private access for unit testing only.
  friend class CompliantContactManagerTest;

  void DoCalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const final {}

  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      multibody::internal::AccelerationKinematicsCache<T>*) const final {}

  void DoCalcDiscreteValues(const drake::systems::Context<T>&,
                            drake::systems::DiscreteValues<T>*) const final {}

  void DoCalcContactResults(const systems::Context<T>&,
                            ContactResults<T>*) const final {}

  void DeclareCacheEntries() final;

  // Returns the point contact stiffness stored in group
  // geometry::internal::kMaterialGroup with property
  // geometry::internal::kPointStiffness for the specified geometry.
  // If not present, it returns MultibodyPlant's default stiffness.
  T GetPointContactStiffness(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Returns the dissipation time constant stored in group
  // geometry::internal::kMaterialGroup with property
  // "dissipation_time_constant". If not present, it returns
  // plant().time_step().
  T GetDissipationTimeConstant(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<T>& inspector) const;

  // Utility to combinee compliances k1 and k2 according to the rule:
  //   k  = k₁⋅k₂/(k₁+k₂)
  // In other words, the combined compliance (the inverse of stiffness) is the
  // sum of the individual compliances.
  static T CombineCompliance(const T& k1, const T& k2);

  // Utility to combine linear dissipation time constants. Consider two
  // spring-dampers with stiffnesses k₁ and k₂, and dissipation time scales τ₁
  // and τ₂, respectively. When these spring-dampers are connected in series,
  // they result in an equivalent spring-damper with stiffness k  =
  // k₁⋅k₂/(k₁+k₂) and dissipation τ = τ₁ + τ₂.
  // This method returns tau1 + tau2.
  static T CombineDissipationTimeConstant(const T& tau1, const T& tau2);

  // Given the configuration stored in `context`, this methods appends discrete
  // pairs corresponding to point contact into `pairs`.
  void AppendDiscreteContactPairsForPointContact(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Given the configuration stored in `context`, this methods appends discrete
  // pairs corresponding to hydroelastic contact into `pairs`.
  void AppendDiscreteContactPairsForHydroelasticContact(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Given the configuration stored in `context`, this method computes all
  // discrete contact pairs, including point and hydroelastic contact, into
  // `pairs.`
  // @pre pairs != nullptr.
  void CalcDiscreteContactPairs(
      const systems::Context<T>& context,
      std::vector<internal::DiscreteContactPair<T>>* pairs) const;

  // Eval version of CalcDiscreteContactPairs().
  const std::vector<internal::DiscreteContactPair<T>>& EvalDiscreteContactPairs(
      const systems::Context<T>& context) const {
    return plant()
        .get_cache_entry(cache_indexes_.discrete_contact_pairs)
        .template Eval<std::vector<internal::DiscreteContactPair<T>>>(context);
  }

  // Given the configuration stored in `context`, this method computes the
  // contact Jacobian cache. See ContactJacobianCache for details.
  void CalcContactJacobianCache(const systems::Context<T>& context,
                                internal::ContactJacobianCache<T>* cache) const;

  // Eval version of CalcContactJacobianCache().
  const internal::ContactJacobianCache<T>& EvalContactJacobianCache(
      const systems::Context<T>& context) const {
    return plant()
        .get_cache_entry(cache_indexes_.contact_jacobian)
        .template Eval<internal::ContactJacobianCache<T>>(context);
  }

  std::unique_ptr<contact_solvers::internal::ContactSolver<T>> contact_solver_;
  CacheIndexes cache_indexes_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
