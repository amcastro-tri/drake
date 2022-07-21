#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapHolonomicConstraint<T>::Parameters::Parameters(
    VectorX<T> lower_limits, VectorX<T> upper_limits,
    VectorX<T> stiffnesses, VectorX<T> relaxation_times, double beta)
    : lower_limits_(std::move(lower_limits)),
      upper_limits_(std::move(upper_limits)),
      stiffnesses_(std::move(stiffnesses)),
      relaxation_times_(std::move(relaxation_times)),
      beta_(beta) {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  DRAKE_DEMAND(lower_limits.size() == upper_limits.size());
  DRAKE_DEMAND(lower_limits.size() == stiffnesses.size());
  DRAKE_DEMAND(lower_limits.size() == relaxation_times.size());
  DRAKE_DEMAND((lower_limits.array() <= kInf).all());
  DRAKE_DEMAND((upper_limits.array() >= -kInf).all());
  DRAKE_DEMAND((lower_limits.array() <= upper_limits.array()).all());
  DRAKE_DEMAND((stiffnesses.array() > 0).all());
  DRAKE_DEMAND((relaxation_times.array() >= 0).all());
}

template <typename T>
SapHolonomicConstraint<T>::SapHolonomicConstraint(int clique, VectorX<T> g,
                                                  MatrixX<T> J,
                                                  Parameters parameters)
    : SapConstraint<T>(clique, std::move(g), std::move(J)),
      parameters_(std::move(parameters)) {
  // N.B. SapConstraint's constructor already checked the sizes of g its
  // Jacobian.
  DRAKE_DEMAND(this->constraint_function().size() ==
               parameters_.num_constraint_equations());
}

template <typename T>
SapHolonomicConstraint<T>::SapHolonomicConstraint(int first_clique,
                                                  int second_clique,
                                                  VectorX<T> g,
                                                  MatrixX<T> J_first_clique,
                                                  MatrixX<T> J_second_clique,
                                                  Parameters parameters)
    : SapConstraint<T>(first_clique, second_clique, std::move(g),
                       std::move(J_first_clique), std::move(J_second_clique)),
      parameters_(std::move(parameters)) {
  // N.B. SapConstraint's constructor already checked the sizes of g and its
  // Jacobian.
  DRAKE_DEMAND(this->constraint_function().size() ==
               parameters_.num_constraint_equations());
}

template <typename T>
VectorX<T> SapHolonomicConstraint<T>::CalcBiasTerm(
    const T& time_step, const T& wi) const {
  using std::min;
  const double beta = parameters_.beta();

  // Relaxation used in the near-rigid regime.
  const T R_near_rigid = beta * beta / (4.0 * M_PI * M_PI) * wi;

  // Relaxation (inverse of) for a constraint without dissipation.
  const VectorX<T> R_elastic_inv =
      time_step * time_step * parameters_.stiffnesses();

  // Default to user supplied relaxation times.
  VectorX<T> relaxation_times = parameters_.relaxation_times();

  // Adjusted relaxation times to the near-rigid regime.
  // In this near-rigid regime a relaxation time equal to the time step leads to
  // a critically damped constraint, [Castro et al., 2022].
  for (int e = 0; e < this->num_constraint_equations(); ++e) {
    // If the elastic relaxation R_elastic is smaller than the near-rigid regime
    // relaxation R_near_rigid, that means that time_step will not be able to
    // resolve the dynamics introduced by this constraint. We call this the
    // "near-rigid" regime. Here we clamp relaxation time to the time step.
    if (R_near_rigid * R_elastic_inv(e) > 1.0) {
      relaxation_times(e) = min(time_step, relaxation_times(e));
    }
  }

  return -this->constraint_function().array() /
         (time_step + relaxation_times.array());
}

template <typename T>
VectorX<T> SapHolonomicConstraint<T>::CalcDiagonalRegularization(
    const T& time_step, const T& wi) const {
  using std::max;

  const double beta = parameters_.beta();

  // Relaxation used in the near-rigid regime.
  const T R_near_rigid = beta * beta / (4.0 * M_PI * M_PI) * wi;

  // Relaxation (inverse of) for a constraint without dissipation.
  const VectorX<T> R_elastic_inv =
      time_step * time_step * parameters_.stiffnesses();

  // Compute relaxation clamped to R_near_rigid.
  VectorX<T> R(this->num_constraint_equations());
  for (int e = 0; e < this->num_constraint_equations(); ++e) {
    // If the elastic relaxation R_elastic is smaller than the near-rigid regime
    // relaxation R_near_rigid, that means that time_step will not be able to
    // resolve the dynamics introduced by this constraint. We call this the
    // "near-rigid" regime.
    if (R_near_rigid * R_elastic_inv(e) > 1.0) {
      const T& k = parameters_.stiffnesses()(e);
      const T& tau = parameters_.relaxation_times()(e);
      const T Re = 1.0 / (time_step * k * (time_step + tau));
      R(e) = max(R_near_rigid, Re);
    }
  }

  return R;
}

template <typename T>
void SapHolonomicConstraint<T>::Project(
    const Eigen::Ref<const VectorX<T>>& y, const Eigen::Ref<const VectorX<T>>&,
    EigenPtr<VectorX<T>> gamma, MatrixX<T>* dPdy) const {
  DRAKE_DEMAND(gamma != nullptr);
  DRAKE_DEMAND(gamma->size() == this->num_constraint_equations());
  // Limits in the impulses.
  const VectorX<T>& gl = parameters_.lower_limits();
  const VectorX<T>& gu = parameters_.upper_limits();

  *gamma = y.array().max(gl.array()).min(gu.array());
  if (dPdy != nullptr) {
    const int nk = this->num_constraint_equations();
    // Resizing is no-op if already the proper size.
    (*dPdy) = MatrixX<T>::Zero(nk, nk);
    for (int i = 0; i < nk; ++i) {
      if (gl(i) < y(i) && y(i) < gu(i)) (*dPdy)(i, i) = 1.0;
    }
  }
}

template <typename T>
std::unique_ptr<SapConstraint<T>>
SapHolonomicConstraint<T>::Clone() const {
  return std::make_unique<SapHolonomicConstraint<T>>(*this);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::
        SapHolonomicConstraint)
