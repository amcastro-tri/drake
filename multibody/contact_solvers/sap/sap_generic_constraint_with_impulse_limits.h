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

/* Implements an arbitrary holonomic constraint for the SAP formulation [Castro
 et al., 2021].

 Constraint kinematics:
  We can write an arbitrary holonomic constraint as g(q, t) = 0, with g(q, t) ∈
  ℝⁿ and n the number of constraint equations.
  This constraint can be written at the velocity level by taking the time
  derivative to obtain
    ġ(q, t) = J⋅v + b = 0
  where J is the contraint's Jacobian, v the vector of generalized velocities of
  the model and b is the bias term b = ∂g/∂t.

 Compliant impulses:
  We will need an impulse for each component in the constraint equation in g(q,
  t) = 0. Here we consider the more general case in which each impulse γᵢ have a
  lower and upper limit, γₗᵢ and γᵤᵢ respectively.

  Constraints in the SAP formulation model a compliant impulse γ according
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
class SapHolonomicConstraint final : public SapConstraint<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapHolonomicConstraint);

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

  SapHolonomicConstraint(int clique, VectorX<T> g, MatrixX<T> J,
                                        Parameters parameters);

  SapHolonomicConstraint(int first_clique, int second_clique,
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
