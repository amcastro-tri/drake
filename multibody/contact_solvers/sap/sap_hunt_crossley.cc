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
SapHuntCrossley<T>::SapHuntCrossley(int clique, MatrixBlock<T> J, const T& x0,
                                    const Parameters& p)
    : SapConstraint<T>(clique, Vector3<T>(0.0, 0.0, -x0), std::move(J)),
      parameters_(p),
      x0_(x0) {
  DRAKE_DEMAND(clique >= 0);
  DRAKE_DEMAND(p.mu >= 0.0);
  DRAKE_DEMAND(p.stiffness > 0.0);
  DRAKE_DEMAND(p.dissipation >= 0.0);
  DRAKE_DEMAND(p.beta > 0.0);
  DRAKE_DEMAND(this->first_clique_jacobian().rows() == 3);
}

template <typename T>
SapHuntCrossley<T>::SapHuntCrossley(int clique0, int clique1, MatrixBlock<T> J0,
                                    MatrixBlock<T> J1, const T& x0,
                                    const Parameters& p)
    : SapConstraint<T>(clique0, clique1, Vector3<T>(0.0, 0.0, -x0),
                       std::move(J0), std::move(J1)),
      parameters_(p),
      x0_(x0) {
  DRAKE_DEMAND(clique0 >= 0);
  DRAKE_DEMAND(clique1 >= 0);
  DRAKE_DEMAND(p.mu >= 0.0);
  DRAKE_DEMAND(p.stiffness > 0.0);
  DRAKE_DEMAND(p.dissipation >= 0.0);
  DRAKE_DEMAND(p.beta > 0.0);
  DRAKE_DEMAND(this->first_clique_jacobian().rows() == 3);
  DRAKE_DEMAND(this->second_clique_jacobian().rows() == 3);
}

template <typename T>
std::unique_ptr<AbstractValue> SapHuntCrossley<T>::DoMakeData(
    const T& time_step,
    const Eigen::Ref<const VectorX<T>>&) const {
  using std::min;

  const T& mu = parameters_.mu;
  const T& d = parameters_.dissipation;
  const double vs = parameters_.vs;

  SapHuntCrossleyData<T> data;
  typename SapHuntCrossleyData<T>::FrozenData& p = data.frozen_data;
  p.dt = time_step;
  p.x0 = x0_;
  p.v_hat = p.x0 / time_step;
  p.v_min = (d == 0.0) ? p.v_hat : min(p.v_hat, 1.0 / d);
  p.mu = mu;
  p.epsilon_soft = vs;

  return AbstractValue::Make(data);
}

template <typename T>
T SapHuntCrossley<T>::CalcHuntCrossleyForce(const T& k, const T& d, const T& x,
                                            const T& xdot) {
  FNC_HEADER();

  if (x <= 0.0) return 0.0;

  const T damping = 1.0 + d * xdot;
  if (damping <= 0.0) return 0.0;

  const T gamma = k * x * damping;

  return gamma;
}

template <typename T>
T SapHuntCrossley<T>::CalcDiscreteHuntCrossleyAntiderivative(
    typename SapHuntCrossleyData<T>::FrozenData& frozen_data,
    const T& vn) const {
  using std::min;

  // With:
  //  - v̂  = x₀/δt
  //  - vd = 1/d
  // vₘ = min(v̂, vd)
  //
  // For v >= vₘ we define N(v; x₀) = 0
  // Then v < vₘ we have:
  //   N(v; x₀) = δt⋅k⋅[d⋅v²⋅(δt⋅v/3−x₀/2)+v⋅(x₀−δt⋅v/2)] + C
  //   N(v; x₀) = δt⋅k⋅[d⋅v²⋅(δt⋅v/3−x₀/2) - (x₀−δt⋅v)²/(2δt)] + C
  //   n(v; x₀) = N'(v; x₀) = k⋅(x₀−δt⋅v)⋅(1-d⋅v)
  // Where the constant of integration C is set so that N(vₘ; x₀) = 0.
  // Notice that with x = x₀−δt⋅v, we have:
  //   N(v; x₀) = δt⋅k⋅[d⋅v²⋅(δt⋅v/3−x₀/2)] - k⋅x²/2 + C
  // And therefore when d = 0 we have:
  //   N(v; x₀) = k⋅x²/2 + C, the elastic component only.

  // Parameters:
  const T& k = parameters_.stiffness;
  const T& d = parameters_.dissipation;
  const T& dt = frozen_data.dt;
  const T& x0 = frozen_data.x0;
  const T& v_min = frozen_data.v_min;

  // Penetration and rate:
  const T xdot = -vn;
  const T x = x0 + dt * xdot;

  PRINT_VAR(x);
  PRINT_VAR(xdot);  

  // Quick exit if v >= vₘ, since then N = 0.
  if (x <= 0.0) return 0.0;
  const T damping = 1.0 + d * xdot;
  if (damping <= 0.0) return 0.0;

  // Integral of n(v; x₀), when C = 0.
  //   N₀(v; x₀) = δt⋅k⋅[d⋅v²⋅(δt⋅v/3−x₀/2)+v⋅(x₀−δt⋅v/2)]
  auto N0 = [&k, &d, &x0, &dt](const T& v) {
    return dt * k *
           (d * v * v * (dt * v / 3.0 - x0 / 2.0) + v * (x0 - dt * v / 2.0));
  };

  // Find constant of integration C such that N(vₘ; x₀) = 0.  
  const T C = -N0(v_min);


  PRINT_VAR(C);

  // Anti-derivative N(v; x₀):
  const T N = N0(vn) + C;

  return N;
}

template <typename T>
T SapHuntCrossley<T>::CalcDiscreteHuntCrossleyImpulse(const T& dt, const T& vn) const {
  // Parameters:
  const T& k = parameters_.stiffness;
  const T& d = parameters_.dissipation;
  const T& x0 = x0_;

  // Penetration and rate:
  const T xdot = -vn;
  const T x = x0 + dt * xdot;

  const T gamma = dt * CalcHuntCrossleyForce(k, d, x, xdot);
  return gamma;
}

template <typename T>
T SapHuntCrossley<T>::CalcDiscreteHuntCrossleyImpulseGradient(
  const T& dt,
    const T& vn) const {
  // Parameters:
  const T& k = parameters_.stiffness;
  const T& d = parameters_.dissipation;
  const T& x0 = x0_;

  // Penetration and rate:
  const T xdot = -vn;
  const T x = x0 + dt * xdot;

  // Quick exit when Gn = 0.
  if (x <= 0.0) return 0.0;
  const T damping = 1.0 + d * xdot;
  if (damping <= 0.0) return 0.0;

  // dn/dv = -δt⋅k⋅[δt + d⋅(x₀-2⋅δt⋅vₙ)]
  const T dn_dvn = -dt * k * (dt + d * (x0 - 2 * dt * vn));

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
  data.z = data.vn - mu * data.vt_soft;
  data.nz = CalcDiscreteHuntCrossleyImpulse(dt, data.z);
  data.Nz = CalcDiscreteHuntCrossleyAntiderivative(data.frozen_data, data.z);

  PRINT_VAR(data.vn);
  PRINT_VAR(data.z);
  PRINT_VAR(data.nz);
  PRINT_VAR(data.Nz);
}

template <typename T>
T SapHuntCrossley<T>::DoCalcCost(const AbstractValue& abstract_data) const {
  const auto& data = abstract_data.get_value<SapHuntCrossleyData<T>>();
  return -data.Nz;  // ell(vc; x0) = -N(z(vc), x0).
}

template <typename T>
void SapHuntCrossley<T>::DoCalcImpulse(const AbstractValue& abstract_data,
                                       EigenPtr<VectorX<T>> gamma) const {
  const auto& data = abstract_data.get_value<SapHuntCrossleyData<T>>();
  const T& mu = data.frozen_data.mu;
  const T& n = data.nz;
  const Vector2<T>& t_soft = data.t_soft;
  const Vector2<T> gt = -mu * n * t_soft;
  *gamma << gt, n;
}

template <typename T>
void SapHuntCrossley<T>::DoCalcCostHessian(const AbstractValue& abstract_data,
                                           MatrixX<T>* G) const {
  const auto& data = abstract_data.get_value<SapHuntCrossleyData<T>>();
  using std::max;

  *G = Matrix3<T>::Zero();
  
  const T& mu = data.frozen_data.mu;
  const T& dt = data.frozen_data.dt;
  const T& epsilon_soft = data.frozen_data.epsilon_soft;
  const T vt_soft = data.vt_soft;
  const T z = data.z;
  const Vector2<T>& t_soft = data.t_soft;

  // n(z) & n'(z)
  const T n = data.nz;
  const T np = CalcDiscreteHuntCrossleyImpulseGradient(dt, z);

  // Projection matrices.
  const Matrix2<T> P = t_soft * t_soft.transpose();
  const Matrix2<T> Pperp = Matrix2<T>::Identity() - P;

  const Matrix2<T> Gt =
      -mu * mu * np * P + mu * n * Pperp / (vt_soft + epsilon_soft);
  const Vector2<T> Gtn = mu * np * t_soft;

  G->template topLeftCorner<2, 2>() = Gt;
  G->template topRightCorner<2, 1>() = Gtn;
  G->template bottomLeftCorner<1, 2>() = Gtn.transpose();
  (*G)(2, 2) = -np;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapHuntCrossley)
