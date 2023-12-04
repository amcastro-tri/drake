#include "drake/multibody/contact_solvers/sap/sap_hunt_crossley.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
//#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
//#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;
#define PRINT_VAR(a) (void)a;
#define PRINT_VARn(a) (void)a;

#if 0
#define FNC_HEADER()                              \
  std::cout << std::string(80, '*') << std::endl; \
  std::cout << __PRETTY_FUNCTION__ << std::endl;
#endif

#define FNC_HEADER() ;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapHuntCrossley<T>::SapHuntCrossley(const T& fe0, const T& vn0,
                                    SapConstraintJacobian<T> J,
                                    const Parameters& p)
    : SapConstraint<T>(std::move(J), {}), parameters_(p), vn0_(vn0) {
  DRAKE_DEMAND(p.mu >= 0.0);
  DRAKE_DEMAND(p.stiffness > 0.0);
  DRAKE_DEMAND(p.dissipation >= 0.0);
  DRAKE_DEMAND(p.beta >= 0.0);
  DRAKE_DEMAND(this->first_clique_jacobian().rows() == 3);
  fe0_ = fe0 - p.stiffness * p.margin;
}

template <typename T>
std::unique_ptr<AbstractValue> SapHuntCrossley<T>::DoMakeData(
    const T& time_step,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) const {
  using std::max;

  const T& mu = parameters_.mu;
  const T& d = parameters_.dissipation;
  const double vs = parameters_.vs;

  // Similar to SAP's sigma parameter.
  const double sigma = parameters_.sigma;

  // Estimate a w_rms guaranteed to be larger than zero.
  const T w_rms = delassus_estimation.norm() / sqrt(3.0);

  // Compute tangent and normal scaling from Delassus estimation, bounded with
  // w_rms to avoid zero values.
  const T wt =
      max(0.5 * (delassus_estimation(0) + delassus_estimation(1)), w_rms);
  //const T wn = max(delassus_estimation(2), w_rms);
  const T Rt = sigma * wt;  // SAP's regularization.

  SapHuntCrossleyData<T> data;
  typename SapHuntCrossleyData<T>::FrozenData& p = data.frozen_data;
  p.dt = time_step;
  p.fe0 = fe0_;
  p.phi0 = -fe0_ / parameters_.stiffness;  // OK, for data collection only.
  const T damping = max(0.0, 1.0 - d * vn0_);
  const T ne0 = max(0.0, time_step * fe0_);
  p.n0 = ne0 * damping;
  p.mu = mu;
  const T sap_vs = mu * Rt * p.n0;
  p.epsilon_soft = max(vs, sap_vs);
  //p.sap_reg = sigma * wn * p.n0;

  //PRINT_VAR(sigma);
  //PRINT_VAR(p.epsilon_soft);
  //PRINT_VAR(wn);
  //PRINT_VAR(wn * p.n0);

  switch (parameters_.model) {
    case ModelType::kConvex:
      break;
    case ModelType::kLagged:
      break;
  }

  return AbstractValue::Make(data);
}

template <typename T>
T SapHuntCrossley<T>::CalcDiscreteHuntCrossleyAntiderivative(
    typename SapHuntCrossleyData<T>::FrozenData& frozen_data,
    const T& vn) const {
  using std::min;

  // With:
  //  - v̂  = x₀/δt
  //  - vd = 1/d vₘ = min(v̂, vd)
  //
  // For v >= vₘ we define N(v; x₀) = 0 Then v < vₘ we have: N(v; x₀) =
  // δt⋅k⋅[d⋅v²⋅(δt⋅v/3−x₀/2)+v⋅(x₀−δt⋅v/2)] + C N(v; x₀) =
  // δt⋅k⋅[d⋅v²⋅(δt⋅v/3−x₀/2) - (x₀−δt⋅v)²/(2δt)] + C n(v; x₀) = N'(v; x₀) =
  // k⋅(x₀−δt⋅v)⋅(1-d⋅v) Where the constant of integration C is set so that
  // N(vₘ; x₀) = 0. Notice that with x = x₀−δt⋅v, we have: N(v; x₀) =
  // δt⋅k⋅[d⋅v²⋅(δt⋅v/3−x₀/2)] - k⋅x²/2 + C And therefore when d = 0 we have:
  // N(v; x₀) = k⋅x²/2 + C, the elastic component only.

  // Version in terms of forces, to avoid x₀, which might go to infinity for
  // some discrete-hydroelastic configurations. N(v; f₀) =
  // δt⋅[d⋅v²⋅(δt⋅k⋅v/3−f₀/2)+v⋅(f₀−δt⋅k⋅v/2)] N(v; f₀) = -δt⋅[d⋅ẋ²⋅(δt⋅k⋅ẋ/3
  // + f₀/2) + ẋ⋅(f₀ + δt⋅k⋅ẋ/2)] N(v; f₀) = -δt⋅[d⋅ẋ²/2⋅(f₀ + 2/3⋅δt⋅k⋅ẋ) +
  // ẋ⋅(f₀ + 1/2⋅δt⋅k⋅ẋ)]

  // Parameters:
  const T& k = parameters_.stiffness;
  const T& d = parameters_.dissipation;
  const T& dt = frozen_data.dt;
  const T& fe0 = frozen_data.fe0;

  // We define the "dissipation" velocity vd at which the dissipation term
  // vanishes using a small tolerance so that vd goes to a large number in the
  // limit to d = 0.
  const T vd = 1.0 / (d + 1.0e-20);

  // Similarly, we define v_hat as the velocity at which the elastic term goes
  // to zero. Using a small tolerance so that it goes to a infinity (a large
  // number) in the limit to small k (e.g. from discrete hydroelastic).
  const T v_hat = fe0 / dt / (k + 1.0e-20);

  // With these tolerances in vd and v_hat, we can define a v_max that goes to a
  // large number in the limit to either d = 0 or k = 0.
  const T v_max = min(v_hat, vd);

  // With these definitions, then the cost can be computed as: N(vn) =
  // N0(min(vn, v_max)), since for large values of v_max we'll correctly get
  // min(vn, v_max) = vn.
  const T xdot = -min(vn, v_max);

  // Integral of n(v; fe₀). N(v; fe₀) = -δt⋅[d⋅ẋ²/2⋅(f₀ + 2/3⋅δt⋅k⋅ẋ) + ẋ⋅(f₀
  //  + 1/2⋅δt⋅k⋅ẋ)] N(v; fe₀) = -δt⋅[d⋅ẋ²/2⋅(f₀ + 2/3⋅Δf) + ẋ⋅(f₀ +
  //  1/2⋅Δf)]; Δf = δt⋅k⋅ẋ
  //
  // N.B. Here we define N0 as the integral of the impulse including the portion
  // of the functional form at which the impulse is negative.

  auto N0 = [&k, &d, &fe0, &dt](const T& x_dot) {
    const T df = dt * k * x_dot;
    return -dt * (d * x_dot * x_dot / 2.0 * (fe0 + 2.0 / 3.0 * df) +
                  x_dot * (fe0 + 1.0 / 2.0 * df));
  };

  return N0(xdot);
}

template <typename T>
T SapHuntCrossley<T>::CalcDiscreteHuntCrossleyImpulse(const T& dt,
                                                      const T& vn) const {
  // Parameters:
  const T& k = parameters_.stiffness;
  const T& d = parameters_.dissipation;
  const T& fe0 = fe0_;

  // Penetration and rate:
  const T xdot = -vn;
  const T fe = fe0 + dt * k * xdot;
  if (fe <= 0.0) return 0.0;
  const T damping = 1.0 + d * xdot;
  if (damping <= 0.0) return 0.0;
  const T gamma = dt * fe * damping;

  return gamma;
}

template <typename T>
T SapHuntCrossley<T>::CalcDiscreteHuntCrossleyImpulseGradient(
    const T& dt, const T& vn) const {
  // Parameters:
  const T& k = parameters_.stiffness;
  const T& d = parameters_.dissipation;
  const T& fe0 = fe0_;

  // Penetration and rate:
  const T xdot = -vn;
  const T fe = fe0 + dt * k * xdot;

  // Quick exits.
  if (fe <= 0.0) return 0.0;
  const T damping = 1.0 + d * xdot;
  if (damping <= 0.0) return 0.0;

  // dn/dv = -δt⋅[k⋅δt⋅(1+d⋅ẋ) + d⋅(fe₀+δt⋅k⋅ẋ)]
  const T dn_dvn = -dt * (k * dt * damping + d * fe);

  return dn_dvn;
}

template <typename T>
void SapHuntCrossley<T>::DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                                    AbstractValue* abstract_data) const {
  auto& data = abstract_data->get_mutable_value<SapHuntCrossleyData<T>>();

  // Parameters:
  const T& mu = data.frozen_data.mu;
  const T& dt = data.frozen_data.dt;
  const T& epsilon_soft = data.frozen_data.epsilon_soft;

  // Computations dependent on vc.
  data.vc = vc;
  data.vn = vc[2];
  data.vt = vc.template head<2>();
  data.vt_soft = SoftNorm(data.vt, epsilon_soft);
  data.t_soft = data.vt / (data.vt_soft + epsilon_soft);
  switch (parameters_.model) {
    case ModelType::kConvex:
      data.z = data.vn - mu * data.vt_soft;
      break;
    case ModelType::kLagged:
      // This effectively evaluates n and N at z = vn for the lagged model.
      data.z = data.vn;
      break;
  };
  data.nz = CalcDiscreteHuntCrossleyImpulse(dt, data.z);
  data.Nz = CalcDiscreteHuntCrossleyAntiderivative(data.frozen_data, data.z);
  data.phi = data.frozen_data.phi0 + dt * data.vn;
/*
  PRINT_VAR(data.vn);
  PRINT_VAR(data.vt_soft);
  PRINT_VAR(data.z);
  PRINT_VAR(data.nz);
  PRINT_VAR(data.Nz);
*/
}

template <typename T>
T SapHuntCrossley<T>::epsilon_soft(const AbstractValue& abstract_data) const {
  const auto& data = abstract_data.get_value<SapHuntCrossleyData<T>>();
  return data.frozen_data.epsilon_soft;
}

template <typename T>
T SapHuntCrossley<T>::phi(const AbstractValue& abstract_data) const {
  const auto& data = abstract_data.get_value<SapHuntCrossleyData<T>>();
  return data.phi;
}

template <typename T>
T SapHuntCrossley<T>::phi0(const AbstractValue& abstract_data) const {
  const auto& data = abstract_data.get_value<SapHuntCrossleyData<T>>();
  return data.frozen_data.phi0;
}

template <typename T>
T SapHuntCrossley<T>::DoCalcCost(const AbstractValue& abstract_data) const {
  const auto& data = abstract_data.get_value<SapHuntCrossleyData<T>>();
  switch (parameters_.model) {
    case ModelType::kConvex:
      return -data.Nz;  // ell(vc; fe0) = -N(z(vc), fe0).
      break;
    case ModelType::kLagged:
      const T& mu = data.frozen_data.mu;
      const T& n0 = data.frozen_data.n0;
      const T& N = data.Nz;
      const T& vt_soft = data.vt_soft;
      return -N + mu * vt_soft * n0;
      break;
  }
  DRAKE_UNREACHABLE();
}

template <typename T>
void SapHuntCrossley<T>::DoCalcImpulse(const AbstractValue& abstract_data,
                                       EigenPtr<VectorX<T>> gamma) const {
  const auto& data = abstract_data.get_value<SapHuntCrossleyData<T>>();
  const T& mu = data.frozen_data.mu;
  const T& n0 = data.frozen_data.n0;
  const T& n = data.nz;
  const Vector2<T>& t_soft = data.t_soft;

  const T n_friction = parameters_.model == ModelType::kConvex ? n : n0;
  const Vector2<T> gt = -mu * n_friction * t_soft;
  *gamma << gt, n;
}

template <typename T>
void SapHuntCrossley<T>::DoCalcCostHessian(const AbstractValue& abstract_data,
                                           MatrixX<T>* G) const {
  const auto& data = abstract_data.get_value<SapHuntCrossleyData<T>>();
  using std::max;

  // Frozen data
  const T& mu = data.frozen_data.mu;
  const T& dt = data.frozen_data.dt;
  const T& epsilon_soft = data.frozen_data.epsilon_soft;
  const T& n0 = data.frozen_data.n0;

  // Data
  const T vt_soft = data.vt_soft;
  const T z = data.z;
  const T& n = data.nz;
  const Vector2<T>& t_soft = data.t_soft;

  // n(z) & n'(z)
  const T np = CalcDiscreteHuntCrossleyImpulseGradient(dt, z);

  // Projection matrices.
  const Matrix2<T> P = t_soft * t_soft.transpose();
  const Matrix2<T> Pperp = Matrix2<T>::Identity() - P;

  Matrix2<T> Gt;
  Vector2<T> Gtn;
  switch (parameters_.model) {
    case ModelType::kConvex:
      Gt = -mu * mu * np * P + mu * n * Pperp / (vt_soft + epsilon_soft);
      Gtn = mu * np * t_soft;
      break;
    case ModelType::kLagged:
      Gt = mu * n0 * Pperp / (vt_soft + epsilon_soft);
      Gtn.setZero();
      break;
  }

  G->template topLeftCorner<2, 2>() = Gt;
  G->template topRightCorner<2, 1>() = Gtn;
  G->template bottomLeftCorner<1, 2>() = Gtn.transpose();
  (*G)(2, 2) = -np;

  // PRINT_VAR(np);
  // PRINT_VARn(*G);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapHuntCrossley)
