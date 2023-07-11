#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
struct SapHuntCrossleyData {
  // Unlike the rest of the data stored in this struct, this data is not a
  // function of constraint velocity vc but it remains const after MakeData().
  struct FrozenData {
    T dt;
    T x0;
    T v_hat;  // v̂  = x₀/δt
    T v_min;  // vₘ = min(v̂, vd), for d > 0 or vₘ = v̂ if d = 0.
    T mu;
    T epsilon_soft;
  };
  FrozenData frozen_data;

  // We store the constraint velocity here for convenience.
  Vector3<T> vc;

  T vn{};
  Vector2<T> vt;
  T vt_soft{};           // Soft norm of vt.
  Vector2<T> t_soft;   // (soft) tangent vector, t_soft = vt / (vt_soft + εₛ).  
  T z{};                 // z = vn - mu * vt_soft.
  T nz{}, Nz{};            // n(z), N(z).
};  

/* Implements contact constraints for the SAP solver.
 Here we provide a brief description of the contact forces modeled by this
 constraint, enough to introduce notation and the constraint's parameters.
 Please refer to [Castro et al., 2021] for an in-depth discussion.

 Normal Compliance:
  Contact constraints in the SAP formulation model a compliant normal impulse γₙ
  according to:
    γₙ/δt = (−k⋅ϕ−d⋅vₙ)₊
  where δt is the time step used in the formulation, k is the contact stiffness
  (in N/m), d is the dissipation (in N⋅s/m) and (x)₊ = max(0, x). ϕ and vₙ are
  the next time step values of the signed distance function and normal velocity
  respectively. ϕ is defined to be negative when bodies overlap and positive
  otherwise. vₙ is defined to be positive when bodies move away from each other
  and negative when they move towards each other. Dissipation is parameterized
  as d = tau_d⋅k, where tau_d is the "dissipation time scale".

 Regularized Friction:
  SAP contact constraints regularize friction. That is, in stiction the
  tangential contact impulses obeys:
    γₜ = −vₜ/Rₜ
  where vₜ is the tangential velocity and Rₜ is the regularization parameter for
  the tangential direction.
  During sliding, the friction impulse obeys:
    γₜ = −μγₙt̂
  where t̂ = vₜ/‖vₜ‖.
  Notice that:
    1. The friction impulse always opposes the tangential velocity, satisfying
       the principle of maximum dissipation.
    2. It obeys Coulomb's law of friction, i.e. ‖γₜ‖ ≤ μγₙ.

  Regularization of friction means that during stiction there might be a
  residual non-zero slip velocity, even if small and negligible for a particular
  application. [Castro et al., 2021] estimate this slip velocity vₛ to be in the
  order of vₛ ≈ σ⋅δt⋅g, where g is the acceleration of gravity (≈9.81m/s² on
  planet Earth) and σ is a dimensionless parameter used to parameterize the
  regularization introduced by the constraint in a scale independent manner.
  Typical values reside in the range σ ∈ (10⁻⁴,10⁻²).

 The contact velocity vc = [vₜ, vₙ] ∈ ℝ³ for this constraint is defined such
 that:
   vc = J⋅v
 where J is the constraint's Jacobian and v is the vector of generalized
 velocities for the cliques involved in the constraint.

 [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
   Unconstrained Convex Formulation of Compliant Contact. Available at
   https://arxiv.org/abs/2110.10107

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapHuntCrossley final : public SapConstraint<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapHuntCrossley);

  /* Numerical parameters that define the constraint. Refer to this class's
   documentation for details. */
  struct Parameters {
    /* Coefficient of friction μ, dimensionless. It must be non-negative. */
    T mu{0.0};
    /* Contact stiffness k, in N/m. It must be strictly positive. */
    T stiffness{0.0};
    /* Hunt & Crossley dissipation d. It must be non-negative. */
    T dissipation{0.0};
    /* Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the contact frequency
     ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. w
     corresponds to a diagonal approximation of the Delassuss operator for
     each contact. See [Castro et al., 2021] for details. */
    double beta{1.0};
    /* Stiction tolerance, in m/s. */
    double vs{1.0e-3};
  };

  /* Constructs a contact constraint for the case in which only a single clique
   is involved. E.g. contact with the world or self-contact.
   @param[in] clique The clique involved in the contact. Must be non-negative.
   @param[in] J The Jacobian, such that vc = J⋅v. It must have three rows or an
   exception is thrown.
   @param[in] phi0 The value of the signed distance at the previous time step.
   @param[in] parameters Constraint parameters. See Parameters for details. */
  SapHuntCrossley(int clique, MatrixBlock<T> J, const T& x0,
                  const Parameters& parameters);

  /* Alternative constructor for a contact constraint involving a single clique
   that takes a dense Jacobian. */
  SapHuntCrossley(int clique, MatrixX<T> J, const T& x0,
                  const Parameters& parameters)
      : SapHuntCrossley(clique, MatrixBlock<T>(std::move(J)), x0, parameters) {}

  /* Constructs a contact constraint for the case in which two cliques
   are involved.
   @param[in] clique0 First clique involved in the contact. Must be
   non-negative.
   @param[in] clique1 Second clique involved in the contact. Must be
   non-negative.
   @param[in] J0 The Jacobian w.r.t. to the first clique's generalized
   velocities. It must have three rows or an exception is thrown.
   @param[in] J1 The Jacobian w.r.t. to the second clique's generalized
   velocities. It must have three rows or an exception is thrown.
   @param[in] phi0 The value of the signed distance at the previous time step.
   @param[in] parameters Constraint parameters. See Parameters for details. */
  SapHuntCrossley(int clique0, int clique1, MatrixBlock<T> J0,
                  MatrixBlock<T> J1, const T& x0, const Parameters& p);

  /* Alternative constructor for a contact constraint involving two cliques
   that takes a dense Jacobian. */
  SapHuntCrossley(int clique0, int clique1, MatrixX<T> J0, MatrixX<T> J1,
                  const T& x0, const Parameters& parameters)
      : SapHuntCrossley(clique0, clique1, MatrixBlock<T>(std::move(J0)),
                        MatrixBlock<T>(std::move(J1)), x0, parameters) {}

  /* Returns the coefficient of friction for this constraint. */
  const T& mu() const { return parameters_.mu; }

  const Parameters& parameters() const { return parameters_; }

  std::unique_ptr<SapConstraint<T>> Clone() const final {
    return std::make_unique<SapHuntCrossley<T>>(*this);
  }

 private:
  static T SoftNorm(const Eigen::Ref<const VectorX<T>>& x, const T& eps) {
    using std::sqrt;
    const T x2 = x.squaredNorm();
    const T soft_norm = sqrt(x2 + eps * eps) - eps;
    return soft_norm;
  }

  std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const override;
  void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                  AbstractValue* data) const override;
  T DoCalcCost(const AbstractValue& data) const override;
  void DoCalcImpulse(const AbstractValue& abstract_data,
                     EigenPtr<VectorX<T>> gamma) const override;
  void DoCalcCostHessian(const AbstractValue& abstract_data,
                         MatrixX<T>* G) const override;

  // Compute gamma = n(x, xdot) for the Hunt & Crossley model.
  // @param x Penetration distance, positive when in penetration.
  // @param xdot Penetration distance time derivative.
  static T CalcHuntCrossleyForce(const T& k, const T& d, const T& x,
                                 const T& xdot);

  // Computes antiderivative N(vn; x0) such that n(vn; x0) = N'(vn; x0).
  T CalcDiscreteHuntCrossleyAntiderivative(
      typename SapHuntCrossleyData<T>::FrozenData& frozen_data,
      const T& vn) const;

  // Computes discrete impulse function n(vn; x0).
  T CalcDiscreteHuntCrossleyImpulse(const T& dt, const T& vn) const;

  // Computes gradient of the discrete impulse function, n'(vn; x0).
  // This returns n'(vn; x0) = dn/dvn <= 0.
  T CalcDiscreteHuntCrossleyImpulseGradient(const T& dt, const T& vn) const;

  Parameters parameters_;
  T x0_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
