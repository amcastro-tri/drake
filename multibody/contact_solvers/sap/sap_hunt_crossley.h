#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"

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
    T fe0;
    T phi0;
    T n0;  // Normal impulse evaluated at previous time step.
    T mu;
    T epsilon_soft;
  };
  FrozenData frozen_data;

  // We store the constraint velocity here for convenience.
  Vector3<T> vc;

  T vn{};
  T phi{};             // Approximation phi = phi0 + dt * vn
  Vector2<T> vt;
  T vt_soft{};           // Soft norm of vt.
  Vector2<T> t_soft;   // (soft) tangent vector, t_soft = vt / (vt_soft + εₛ).
  T z{};                 // z = vn - mu * vt_soft.
  T nz{}, Nz{};            // n(z), N(z).
};

/* Implements convex model of Hunt & Crossley with regularized friction. For the
 understanding of model parameters and the model of regularized friction, refer
 to [Castro et al., 2020] for the non-convex version of the model.

 [Castro et al., 2020] Castro, A.M., Qu, A., Kuppuswamy, N., Alspach, A. and
 Sherman, M., 2020. A transition-aware method for the simulation of compliant
 contact with regularized friction. IEEE Robotics and Automation Letters, 5(2),
 pp.1859-1866.

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapHuntCrossley final : public SapConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
    Protected copy construction is enabled for sub-classes to use in their
    implementation of DoClone(). */
  //@{
  SapHuntCrossley& operator=(const SapHuntCrossley&) =
      delete;
  SapHuntCrossley(SapHuntCrossley&&) = delete;
  SapHuntCrossley& operator=(SapHuntCrossley&&) = delete;
  //@}

  enum class ModelType {
    kConvex = 0,
    kLagged = 1,
  };

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
    /* Similar to SAP's sigma parameter. */
    double sigma{0.0};  // Typically 1.0e-3 for SAP.
    /* Stiction tolerance, in m/s. */
    double vs{1.0e-3};
    double margin{0.0};  // Geometric margin.
    ModelType model{ModelType::kConvex};
  };

  /* Constructs a contact constraint for the case in which only a single clique
   is involved. E.g. contact with the world or self-contact.
   @param[in] clique The clique involved in the contact. Must be non-negative.
   @param[in] J The Jacobian, such that vc = J⋅v. It must have three rows or an
   exception is thrown.
   @param[in] fe0 The value of the elastic component of force at the previous
   time step.
   @param[in] parameters Constraint parameters. See Parameters for details. */
  SapHuntCrossley(const T& fe0, const T& vn0, SapConstraintJacobian<T> J,
                  const Parameters& parameters);

  /* Returns the coefficient of friction for this constraint. */
  const T& mu() const { return parameters_.mu; }

  const Parameters& parameters() const { return parameters_; }

  T epsilon_soft(const AbstractValue& abstract_data) const;

  T phi(const AbstractValue& abstract_data) const;
  T phi0(const AbstractValue& abstract_data) const;

 private:
  /* Private copy construction is enabled to use in the implementation of
    DoClone(). */
  SapHuntCrossley(const SapHuntCrossley&) = default;

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
  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapHuntCrossley<T>>(new SapHuntCrossley<T>(*this));
  }

  // Computes antiderivative N(vn; fe0) such that n(vn; fe0) = N'(vn; fe0).
  T CalcDiscreteHuntCrossleyAntiderivative(
      typename SapHuntCrossleyData<T>::FrozenData& frozen_data,
      const T& vn) const;

  // Computes discrete impulse function n(vn; fe0).
  T CalcDiscreteHuntCrossleyImpulse(const T& dt, const T& vn) const;

  // Computes gradient of the discrete impulse function, n'(vn; fe0).
  // This returns n'(vn; fe0) = dn/dvn <= 0.
  T CalcDiscreteHuntCrossleyImpulseGradient(const T& dt, const T& vn) const;

  Parameters parameters_;
  T fe0_;
  T vn0_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
