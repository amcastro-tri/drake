#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// TODO(amcastro-tri): update reference [Castro et al., 2022] to the follow up
// paper on arbitrary constraints.

/* Structure to store data needed for SapLimitConstraint computations. */
template <typename T>
struct SapHolonomicConstraintData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapHolonomicConstraintData);

  /* Constructs data for a SapHolonomicConstraintData.
     @param R Regularization parameters.
     @param v_hat Bias term. */
  SapHolonomicConstraintData(VectorX<T> R, VectorX<T> v_hat) {
    const int nk = R.size();
    parameters_.R.resize(nk);
    parameters_.R_inv.resize(nk);
    parameters_.v_hat.resize(nk);
    parameters_.R = R;
    parameters_.R_inv = R.cwiseInverse();
    parameters_.v_hat = v_hat;
    vc_.resize(nk);
    y_.resize(nk);
    gamma_.resize(nk);
    dPdy_.resize(nk, nk);
  }

  /* Regularization R. */
  const VectorX<T>& R() const { return parameters_.R; }

  /* Inverse of the regularization, R⁻¹. */
  const VectorX<T>& R_inv() const { return parameters_.R_inv; }

  /* Constraint bias. */
  const VectorX<T>& v_hat() const { return parameters_.v_hat; }

  /* Const access. */
  const VectorX<T>& vc() const { return vc_; }
  const VectorX<T>& y() const { return y_; }
  const VectorX<T>& gamma() const { return gamma_; }
  const MatrixX<T>& dPdy() const { return dPdy_; }

  /* Mutable access. */
  VectorX<T>& mutable_vc() { return vc_; }
  VectorX<T>& mutable_y() { return y_; }
  VectorX<T>& mutable_gamma() { return gamma_; }
  MatrixX<T>& mutable_dPdy() { return dPdy_; }

 private:
  struct ConstParameters {
    VectorX<T> R;  // Regularization R.
    VectorX<T> R_inv;
    VectorX<T> v_hat;  // Constraint velocity bias.
  };
  ConstParameters parameters_;

  VectorX<T> vc_;
  VectorX<T> y_;      // Un-projected impulse y = −R⁻¹⋅(vc−v̂)
  VectorX<T> gamma_;  // Impulse.
  MatrixX<T> dPdy_;   // Gradient of the projection γ = P(y) w.r.t. y.
};

/* Implements an arbitrary holonomic constraint for the SAP formulation [Castro
 et al., 2022].

 Constraint kinematics:
  We can write an arbitrary holonomic constraint as g(q, t) = 0, with g(q, t) ∈
  ℝⁿ and n the number of constraint equations.
  This constraint can be written at the velocity level by taking the time
  derivative to obtain
    ġ(q, t) = J⋅v + b = 0
  where J is the constraint's Jacobian, v the vector of generalized velocities
  of the model and b is the bias term b = ∂g/∂t.

 Compliant impulses:
  We will need an impulse for each component in the constraint equation in g(q,
  t) = 0. Here we consider the more general case in which each impulse γᵢ (Greek
  letter gamma) is constrained to live in the (convex) set 𝒞ᵢ = [γₗᵢ, γᵤᵢ]
  where γₗᵢ and γᵤᵢ are the lower and upper bounds, respectively.

  Constraints in the SAP formulation model a compliant impulse γ according to:
    y/δt = −k⋅(g+τ⋅ġ)
    γ/δt = P(y)
  where we use the Roman character y for the "unprojected impulses",
  δt is the time step used in the formulation, k is the constraint stiffness (in
  N/m), τ is the dissipation relaxation time (in seconds) and P(y) is a
  projection into the (convex) set 𝒞ᵢ. In this case the projection can
  trivially be computed analytically as:
    P(y) = max(γₗ, min(γᵤ, y))
  independent of the compliant regularization.

  On SAP regularization and bias:
   Here we provide details on the computation of the regularization terms R
   performed by CalcDiagonalRegularization() and the velocity bias v̂ performed
   by CalcBiasTerm(). SAP approximates the constraint fuction as:
     g(v) ≈ g₀ + δt⋅ġ(v) = g₀ + δt⋅(J⋅v + b)
   With this approximation the unprojected impulses y(v) = −δt⋅k⋅(g + τ⋅ġ) can
   be written as:
     y(v) = −R⁻¹⋅(J⋅v − v̂)
   with the regularization R defined as:
     R⁻¹ = δt⋅(δt + τ)⋅k
   and the velocity bias v̂ as:
     v̂ = −g₀/(δt + τ) − b

 [Castro et al., 2022] Castro A., Permenter F. and Han X., 2021. An
   Unconstrained Convex Formulation of Compliant Contact. Available at
   https://arxiv.org/abs/2110.10107

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
     @param impulse_lower_limits vector of lower limits γₗ.
     @param impulse_upper_limits vector of upper limits γᵤ.
     @param stiffnesses vector of stiffnesses kᵢ for each constraint.
     @param relaxation_times vector of relaxation times τᵢ for each constraint.
     @param beta Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the
     constraint frequency ωᵢ for the i-th constraint is below the limit ωᵢ ≤
     2π/δt. That is, the period is limited to Tᵢ = β⋅δt. w corresponds to a
     diagonal approximation of the Delassuss operator for the constraint. See
     [Castro et al., 2022] for details.

     @pre impulse_lower_limits, impulse_upper_limits, stiffnesses and
     relaxation_times must all have the same size.
     @pre impulse_lower_limits <= +∞, componentwise.
     @pre impulse_upper_limits >= -∞, componentwise.
     @pre lower_limit <= upper_limit, componentwise.
     @pre stiffnesses > 0, componentwise.
     @pre relaxation_times >= 0, componentwise
     @pre beta > 0 */
    Parameters(VectorX<T> impulse_lower_limits, VectorX<T> impulse_upper_limits,
               VectorX<T> stiffnesses, VectorX<T> relaxation_times,
               double beta = 0.1);

    const VectorX<T>& impulse_lower_limits() const {
      return impulse_lower_limits_;
    }
    const VectorX<T>& impulse_upper_limits() const {
      return impulse_upper_limits_;
    }
    const VectorX<T>& stiffnesses() const { return stiffnesses_; }
    const VectorX<T>& relaxation_times() const { return relaxation_times_; }
    double beta() const { return beta_; }
    int num_constraint_equations() const {
      return impulse_lower_limits_.size();
    }

   private:
    VectorX<T> impulse_lower_limits_;
    VectorX<T> impulse_upper_limits_;
    VectorX<T> stiffnesses_;
    VectorX<T> relaxation_times_;
    double beta_{0.1};
  };

  /* Constructs a holonomic constraint involving a single clique. The bias term
   b is zero.
   @param[in] clique The clique involved in the constraint.
   @param[in] g The value of the constraint function.
   @param[in] J The Jacobian w.r.t. to the clique's generalized velocities.
   @param[in] parameters Constraint parameters. See Parameters for details.

   @pre clique is non-negative.
   @pre g.size() == J.rows() == parameters.num_constraint_equations(). */
  SapHolonomicConstraint(int clique, VectorX<T> g, MatrixBlock<T> J,
                         Parameters parameters);

  /* Alternative holonomic constraint constructor for a single clique that takes
   a dense Jacobian and zero bias. */
  SapHolonomicConstraint(int clique, VectorX<T> g, MatrixX<T> J,
                         Parameters parameters)
      : SapHolonomicConstraint(clique, std::move(g),
                               MatrixBlock<T>(std::move(J)), parameters) {}

  /* Constructs a holonomic constraint involving two cliques. The bias term b is
   zero.
   @param[in] first_clique First clique involved in the constraint.
   @param[in] second_clique Second clique involved in the constraint.
   @param[in] g The value of the constraint function.
   @param[in] J_first_clique The Jacobian w.r.t. to the first clique's
   generalized velocities.
   @param[in] J_second_clique The Jacobian w.r.t. to the second clique's
   generalized velocities.
   @param[in] parameters Constraint parameters. See Parameters for details.

   @pre first_clique and second_clique are non-negative.
   @pre g.size() == J_first_clique.rows() == J_second_clique.rows() ==
   parameters.num_constraint_equations(). */
  SapHolonomicConstraint(int first_clique, int second_clique, VectorX<T> g,
                         MatrixBlock<T> J_first_clique,
                         MatrixBlock<T> J_second_clique, Parameters parameters);

  /* Alternative holonomic constraint constructor for two cliques that takes
   dense Jacobians and zero bias. */
  SapHolonomicConstraint(int first_clique, int second_clique, VectorX<T> g,
                         MatrixX<T> J_first_clique, MatrixX<T> J_second_clique,
                         Parameters parameters)
      : SapHolonomicConstraint(first_clique, second_clique, std::move(g),
                               MatrixBlock<T>(std::move(J_first_clique)),
                               MatrixBlock<T>(std::move(J_second_clique)),
                               parameters) {}

  /* Single clique holonomic constraints with non-zero bias.
   @param[in] clique The clique involved in the constraint.
   @param[in] g The value of the constraint function.
   @param[in] J The Jacobian w.r.t. to the clique's generalized velocities.
   @param[in] b The bias term, such that ġ = J⋅v + b.
   @param[in] parameters Constraint parameters. See Parameters for details.

   @pre clique is non-negative.
   @pre g.size() == J.rows() == parameters.num_constraint_equations(). */
  SapHolonomicConstraint(int clique, VectorX<T> g, MatrixBlock<T> J,
                         VectorX<T> b, Parameters parameters);

  /* Alternative holonomic constraint constructor for single clique that takes
   a dense Jacobian and non-zero bias. */
  SapHolonomicConstraint(int clique, VectorX<T> g, MatrixX<T> J, VectorX<T> b,
                         Parameters parameters)
      : SapHolonomicConstraint(clique, std::move(g),
                               MatrixBlock<T>(std::move(J)), std::move(b),
                               parameters) {}

  const Parameters& parameters() const { return parameters_; }

  /* Returns the holonomic constraint bias b. */
  const VectorX<T>& bias() const { return bias_; }

  std::unique_ptr<SapConstraint<T>> Clone() const final;

 private:
  void Project(const Eigen::Ref<const VectorX<T>>& y,
               EigenPtr<VectorX<T>> gamma, MatrixX<T>* dPdy = nullptr) const;

  std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const override;
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                  AbstractValue* data) const override;
  T DoCalcCost(const AbstractValue& abstract_data) const override;
  void DoCalcImpulse(const AbstractValue& abstract_data,
                     EigenPtr<VectorX<T>> gamma) const override;
  void DoCalcCostHessian(const AbstractValue& abstract_data,
                         MatrixX<T>* G) const override;

  Parameters parameters_;
  VectorX<T> bias_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
