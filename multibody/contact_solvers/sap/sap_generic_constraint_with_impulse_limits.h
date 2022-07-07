#pragma once

#include <limits>
#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Implements limit constraints for the SAP solver, [Castro et al., 2021]. This
 constraint is used to impose a (compliant, see below) limit on the i-th degree
 of freedom (DOF) of a given clique in a SapContactModel. This constraint
 assumes that the rate of change of the i-th configuration exactly equals its
 corresponding generalized velocities, i.e. q̇ᵢ = vᵢ. This is specially true for
 1-DOF joints such as revolute and prismatic.

 Constraint kinematics:
  We consider the i-th DOF of a clique with m DOFs.
  We denote the configuration with qᵢ its lower limit with qₗ and its upper
  limit with qᵤ, where qₗ < qᵤ. The limit constraint defines a constraint
  function g(q) ∈ ℝ² as:
    g = |qᵢ - qₗ|   for the lower limit and,
        |qᵤ - qᵢ|   for the upper limit
  such that g(qᵢ) < 0 (componentwise) if the constraint is violated. The
  constraint velocity therefore is:
    ġ = | q̇ᵢ|
        |-q̇ᵢ|
  And therefore the constraint Jacobian is:
    J = | eᵢᵀ|
        |-eᵢᵀ|
  where eᵢ is the i-th element of the standard basis of ℝᵐ whose components are
  all zero, except for the i-th component that equals to 1.

  If one of the limits is infinite (qₗ = -∞ or qᵤ = +∞) then only one of the
  equations is considered (the one with finite bound) and g(q) ∈ ℝ i.e.
  num_constraint_equations() = 1.

 Compliant impulse:
  Limit constraints in the SAP formulation model a compliant impulse γ according
  to:
    y/dt = −k⋅g−d⋅ġ
    γ/δt = P(y)
    P(y) = (y)₊
  where δt is the time step used in the formulation, k is the constraint
  stiffness (in N/m), d is the dissipation (in N⋅s/m) and (x)₊ = max(0, x),
  componentwise. Dissipation is parameterized as d = tau_d⋅k, where tau_d is the
  "dissipation time scale". Notice that these impulses are positive when the
  constraint is active and zero otherwise.
  For this constraint the components of γ = [γₗ, γᵤ]ᵀ are constrained to live in
  ℝ⁺ and therefore the projection can trivially be computed analytically as
  P(y) = (y)₊, independent of the compliant regularization, see [Todorov, 2014].

 [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
   Unconstrained Convex Formulation of Compliant Contact. Available at
   https://arxiv.org/abs/2110.10107
 [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
   dynamics with contacts and constraints: Theory and implementation in mujoco.
   In 2014 IEEE International Conference on Robotics and Automation (ICRA) (pp.
   6054-6061). IEEE.

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapGenericConstraintWithImpulseLimits final : public SapConstraint<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapGenericConstraintWithImpulseLimits);

  /* Numerical parameters that define the constraint. Refer to this class's
   documentation for details. */
  class Parameters {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Parameters);

    /* Constructs a valid set of parameters.
     @pre lower_limit < +∞
     @pre upper_limit > -∞
     @pre at least one of lower_limit and upper_limit is finite.
     @pre lower_limit <= upper_limit
     @pre stiffness > 0
     @pre dissipation_time_scale >= 0
     @pre beta > 0 */
    Parameters(VectorX<T> lower_limits, VectorX<T> upper_limits,
               VectorX<T> stiffnesses, VectorX<T> relaxation_times,
               double beta = 0.1);

    const VectorX<T>& lower_limits() const { return lower_limits_; }
    const VectorX<T>& upper_limits() const { return upper_limits_; }
    const VectorX<T>& stiffnesses() const { return stiffnesses_; }
    const VectorX<T>& relaxation_times() const { return relaxation_times_; }
    double beta() const { return beta_; }

   private:
    VectorX<T> lower_limits_;
    VectorX<T> upper_limits_;
    /* Contact stiffness k, in N/m. It must be strictly positive. */
    VectorX<T> stiffnesses_;
    /* Dissipation time scale tau_d, in seconds. It must be non-negative. */
    VectorX<T> relaxation_times_;
    /* Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the constraint
     frequency ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ =
     β⋅δt. w corresponds to a diagonal approximation of the Delassuss operator
     for each contact. See [Castro et al., 2021] for details. */
    double beta_{0.1};
  };

  SapGenericConstraintWithImpulseLimits(int clique, VectorX<T> g, MatrixX<T> J,
                                        Parameters parameters);

  SapGenericConstraintWithImpulseLimits(int first_clique, int second_clique,
                                        VectorX<T> g, MatrixX<T> J_first_clique,
                                        MatrixX<T> J_second_clique,
                                        Parameters parameters);

  const Parameters& parameters() const { return parameters_; }

  /* Implements the projection operation P(y) = min(γₗ, max(γᵤ, y)), independent
   of the regularization R. Refer to SapConstraint::Project() for details. */
  void Project(const Eigen::Ref<const VectorX<T>>& y,
               const Eigen::Ref<const VectorX<T>>& R,
               EigenPtr<VectorX<T>> gamma,
               MatrixX<T>* dPdy = nullptr) const final;

  /* Computes bias term. Refer to SapConstraint::CalcBiasTerm() for details. */
  // TODO(amcastro-tri): Extend SapConstraint so that wi can be a vector with an
  // entry for each constraint equation.
  VectorX<T> CalcBiasTerm(const T& time_step, const T& wi) const final;

  /* Computes the diagonal of the regularization matrix (positive diagonal) R.
   This computes R = [Ri, Ri] (or R = [Ri] if only one limit is imposed) with
     Ri = max(β²/(4π²)⋅wᵢ, (δt⋅(δt+tau_d)⋅k)⁻¹),
   Refer to [Castro et al., 2021] for details. */
  VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                        const T& wi) const final;

  std::unique_ptr<SapConstraint<T>> Clone() const final;

 private:
  Parameters parameters_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
