#include "drake/multibody/contact_solvers/sap/sap_lagged_hunt_crossley.h"

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

#define FNC_HEADER();

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
T bounded_exp(const T& x) {
  using std::exp;
  return exp(x);
#if 0  
  const T x_bound = 10.0;
  const T exp_bound = exp(x_bound);
  const T dexp_bound = exp_bound;
  if (x < x_bound) return exp(x);
  return exp_bound + dexp_bound * (x - x_bound);
#endif  
}

template <typename T>
SapLaggedHuntCrossley<T>::SapLaggedHuntCrossley(int clique, MatrixX<T> J,
                                                const T& x0, const T& xdot0,
                                                const Parameters& p)
    : SapConstraint<T>(clique, Vector3<T>(0.0, 0.0, -x0), std::move(J)),
      parameters_(p),
      x0_(x0),
      xdot0_(xdot0) {
  DRAKE_DEMAND(clique >= 0);
  DRAKE_DEMAND(p.mu >= 0.0);
  DRAKE_DEMAND(p.stiffness > 0.0);
  DRAKE_DEMAND(p.hunt_crossley_dissipation >= 0.0);
  DRAKE_DEMAND(p.beta > 0.0);
  DRAKE_DEMAND(this->first_clique_jacobian().rows() == 3);
}

template <typename T>
SapLaggedHuntCrossley<T>::SapLaggedHuntCrossley(int clique0, int clique1,
                                                MatrixX<T> J0, MatrixX<T> J1,
                                                const T& x0, const T& xdot0,
                                                const Parameters& p)
    : SapConstraint<T>(clique0, clique1, Vector3<T>(0.0, 0.0, -x0),
                       std::move(J0), std::move(J1)),
      parameters_(p),
      x0_(x0),
      xdot0_(xdot0) {
  DRAKE_DEMAND(clique0 >= 0);
  DRAKE_DEMAND(clique1 >= 0);
  DRAKE_DEMAND(p.mu >= 0.0);
  DRAKE_DEMAND(p.stiffness > 0.0);
  DRAKE_DEMAND(p.hunt_crossley_dissipation >= 0.0);
  DRAKE_DEMAND(p.beta > 0.0);
  DRAKE_DEMAND(this->first_clique_jacobian().rows() == 3);
  DRAKE_DEMAND(this->second_clique_jacobian().rows() == 3);
}

template <typename T>
void SapLaggedHuntCrossley<T>::DoInitializeParameters(
    const T& dt,
    const Eigen::Ref<const VectorX<T>>& delassus_estimation) {
  using std::max;
  using std::min;
  using std::sqrt;
  FNC_HEADER();

  // Physical parameters.
  const T& k = parameters_.stiffness;
  const T& d = parameters_.hunt_crossley_dissipation;
  const T& xe = parameters_.delta;
  const T& x0 = x0_;

  // Regularization parameters
  const double vs = parameters_.vs;
  const double beta = parameters_.beta;

  // Estimate a w_rms guaranteed to be larger than zero.
  const T w_rms = delassus_estimation.norm() / sqrt(3.0);
  const T wn = max(delassus_estimation(2), w_rms);

  // Post initialization parameters.
  epsilon_soft_ = vs / sqrt(3.0);
  const double beta_factor = 4.0 * M_PI * M_PI / (beta * beta);
  kMax_ = beta_factor / (dt * dt * wn);
  kEff_ = min(kMax_, k);
  time_step_ = dt;
  kappa_ = kEff_ * xe * xe;  // Characteristic energy.

  // Incremental potential is zero for x < x_min
  x_min_ = (d == 0.0) ? 0.0 : max(0.0, x0 - dt / d);

  PRINT_VAR(epsilon_soft_);
  PRINT_VAR(kMax_);
  PRINT_VAR(k);
  PRINT_VAR(kEff_);
  PRINT_VAR(kappa_);
  PRINT_VAR(x0_);
  PRINT_VAR(x_min_);
}

template <typename T>
T SapLaggedHuntCrossley<T>::CalcDiscreteHuntCrossleyCost(const T& vn) const {
    using std::max;
  // TODO: consider a "bounded exp" that is linear after a given threshold to
  // avoid numerical issues.
  using std::exp;
  // Physical parameters.
  //const T& k = kEff_;
  const T& d = parameters_.hunt_crossley_dissipation;
  const T& xe = parameters_.delta;
  const T lambda = lambda_;
  const T& dt = time_step_;
  const T& x0 = x0_;
  const T& x_min = x_min_;
  //const T& xdot0 = xdot0_;

  const T xdot = -vn;
  const T x = x0 + dt * xdot;

  // B(u < u_min) = 0.
  if (x <= x_min) return 0.0;

  const T damping = 1.0 + d * xdot;
  if (damping < 0.0) return 0.0;

  // Dimensionless parameters.  
  const T b = 1.0 - d / dt * x0;
  const T c = d / dt * xe;
  const T lambda2 = lambda * lambda;
  const T lambda3 = lambda * lambda2;  

  auto dimensionless_potential = [&](const T& u) {
    const T Bu =
        (lambda2 * (c * u + b) * u - lambda * (2.0 * c * u + b) + 2.0 * c) *
        bounded_exp(lambda * u) / lambda3;
    return Bu;
  };

  // We define the constant of integration so that B(u_min) = 0
  const T u_min = x_min / xe;
  const T C = -dimensionless_potential(u_min);

  const T u = x / xe;
  const T Bu = dimensionless_potential(u) + C;

  const T ell = kappa_ * Bu;
  return ell;
}

template <typename T>
T SapLaggedHuntCrossley<T>::CalcHuntCrossleyImpulse(const T& x,
                                                    const T& xdot) const {
    using std::max;                                                        
  FNC_HEADER();    
  // Physical parameters.
  const T& k = kEff_;
  const T& d = parameters_.hunt_crossley_dissipation;
  const T& xe = parameters_.delta;
  const T lambda = lambda_;
  const T& dt = time_step_;
  const T& x_min = x_min_;

  if (x < x_min) return 0.0;

  const T damping = 1.0 + d * xdot;
  if (damping < 0.0) return 0;

  const T s = x / xe;
  const T bs = s * bounded_exp(lambda * s);
  const T gamma_ref = dt * k * xe;  
  const T gamma = gamma_ref * bs * damping;
  
  PRINT_VAR(x);
  PRINT_VAR(xdot);
  PRINT_VAR(s);
  PRINT_VAR(bs);
  PRINT_VAR(gamma);

  return gamma;
}

template <typename T>
T SapLaggedHuntCrossley<T>::CalcDiscreteHuntCrossleyImpulse(const T& vn) const {
  const T xdot = -vn;
  const T x = x0_ + time_step_ * xdot;
  return CalcHuntCrossleyImpulse(x, xdot);
}

template <typename T>
T SapLaggedHuntCrossley<T>::CalcDiscreteHuntCrossleyImpulseGradient(
    const T& vn) const {
  // Physical parameters.
  const T& k = kEff_;
  const T& d = parameters_.hunt_crossley_dissipation;
  const T& xe = parameters_.delta;
  const T lambda = lambda_;
  const T& dt = time_step_;
  const T& x_min = x_min_;

  // Penetration and rate.
  const T& xdot = -vn;
  const T x = x0_ + time_step_ * xdot;

  // Determine if Gn is zero.
  const T damping = 1.0 + d * xdot;
  T Gn = 0.0;
  if (x > x_min && damping > 0.0) {
    const T s = x / xe;
    const T exp_s = bounded_exp(lambda * s);
    const T bs = s * exp_s;
    const T bp = (1.0 + lambda * s) * exp_s;
    // N.B. Gn = -dn/dvn > 0
    Gn = dt * dt * k * (bp * damping + xe * d / dt * bs);
  }
  return Gn;
}

template <typename T>
T SapLaggedHuntCrossley<T>::DoCalcCost(
    const Eigen::Ref<const VectorX<T>>& vc) const {
  //FNC_HEADER();      
  const T& mu = parameters_.mu;      
  const T& vn = vc[2];
  const auto vt = vc.template head<2>();
  const T vt_soft = SoftNorm(vt, epsilon_soft_);
  const T z = vn - mu * vt_soft;
  // ell(v) = -N(z)
  const T N = CalcDiscreteHuntCrossleyCost(z);
  return N;
}

template <typename T>
void SapLaggedHuntCrossley<T>::DoCalcImpulse(
    const Eigen::Ref<const VectorX<T>>& vc, EigenPtr<VectorX<T>> gamma) const {  
  const T& mu = parameters_.mu;
  const T& vn = vc[2];  
  const auto vt = vc.template head<2>();
  const T vt_soft = SoftNorm(vt, epsilon_soft_);
  const T z = vn - mu * vt_soft;
  const T n = CalcDiscreteHuntCrossleyImpulse(z);  // = n(z).
  const Vector2<T> t_soft = vt / (vt_soft + epsilon_soft_);  
  const Vector2<T> gt = -mu * n * t_soft;
  *gamma << gt, n;
  PRINT_VAR(gamma->transpose());
}

template <typename T>
void SapLaggedHuntCrossley<T>::DoCalcCostHessian(
    const Eigen::Ref<const VectorX<T>>& vc, MatrixX<T>* G) const {
  using std::max;
  FNC_HEADER();

  const T& mu = parameters_.mu;

  *G = Matrix3<T>::Zero();

  // Penetration and rate.
  const T& vn = vc[2];
  const auto vt = vc.template head<2>();
  const T vt_soft = SoftNorm(vt, epsilon_soft_);
  const T z = vn - mu * vt_soft;

  PRINT_VAR(z);
  PRINT_VAR(vn);
  PRINT_VAR(vt_soft);

  // n(z) & n'(z)
  const T n = CalcDiscreteHuntCrossleyImpulse(z);
  const T np = CalcDiscreteHuntCrossleyImpulseGradient(z);
  const T dn_dvn = -np;

  // Soft tangent vector.
  const Vector2<T> t_soft = vt / (vt_soft + epsilon_soft_);

  // Projection matrices.
  const Matrix2<T> P = t_soft * t_soft.transpose();
  const Matrix2<T> Pperp = Matrix2<T>::Identity() - P;

  const Matrix2<T> Gt =
      -mu * mu * dn_dvn * P + mu * n * Pperp / (vt_soft + epsilon_soft_);

  PRINT_VAR(n);
  PRINT_VAR(dn_dvn);
  PRINT_VAR(t_soft.transpose());
  PRINT_VARn(P);
  PRINT_VARn(Gt);

  const Vector2<T> Gtn = mu * dn_dvn * t_soft;      

  G->template topLeftCorner<2,2>() = Gt;
  G->template topRightCorner<2,1>() = Gtn;
  G->template bottomLeftCorner<1,2>() = Gtn.transpose();
  (*G)(2, 2) = -dn_dvn;
  PRINT_VARn(*G);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapLaggedHuntCrossley)
