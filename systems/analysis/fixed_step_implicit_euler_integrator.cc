#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/fixed_step_implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

namespace drake {
namespace systems {

template <class T>
void FixedStepImplicitEulerIntegrator<T>::IterationMatrix::
    SetAndFactor(const MatrixX<T>& iteration_matrix) {
  INSTRUMENT_FUNCTION("Jacobian factorization.");
  LU_.compute(iteration_matrix);
  matrix_factored_ = true;
}

template <class T>
VectorX<T> FixedStepImplicitEulerIntegrator<T>::IterationMatrix::Solve(
    const VectorX<T>& b) const {
  return LU_.solve(b);
}

template <class T>
void FixedStepImplicitEulerIntegrator<T>::DoResetStatistics() {
  statistics_ = {};
}

template <class T>
void FixedStepImplicitEulerIntegrator<T>::CalcJacobian(const T& t,
    const VectorX<T>& x, MatrixX<T>* J) {
  INSTRUMENT_FUNCTION("Jacobian compuation.");    

  // Get the current number of ODE evaluations.
  int64_t current_derivative_evals = this->get_num_derivative_evaluations();

  switch (jacobian_scheme_) {
    case JacobianComputationScheme::kForwardDifference:
      ComputeForwardDiffJacobian(t, x, J);
      break;

    case JacobianComputationScheme::kCentralDifference:
      throw std::logic_error("not implemented");
      // ComputeCentralDiffJacobian(system, t, x, &*context, &J);
      break;

    case JacobianComputationScheme::kAutomatic:
      // ComputeAutoDiffJacobian(system, t, x, *context, &J);
      throw std::logic_error("not implemented");
      break;
  }

  // Update statistics.
  statistics_.num_jacobian_reforms++;

  // Use the new number of ODE evaluations to determine the number of Jacobian
  // evaluations.
  statistics_.num_jacobian_function_evaluations +=
      this->get_num_derivative_evaluations() - current_derivative_evals;
}

template <class T>
void FixedStepImplicitEulerIntegrator<T>::ComputeForwardDiffJacobian(
    const T& t, const VectorX<T>& x,
    MatrixX<T>* J) {
  using std::abs;
  using std::max;  

  // Set epsilon to the square root of machine precision.
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Get the number of continuous state variables xt.
  const int n = x.size();

  DRAKE_LOGGER_DEBUG(
      "  ImplicitIntegrator Compute Forwarddiff {}-Jacobian t={}", n, t);
  DRAKE_LOGGER_DEBUG(
      "  computing from state {}", fmt_eigen(xt.transpose()));

  // Initialize the Jacobian.
  J->resize(n, n);

  // system and scratch context.
  // N.B. The context can have garbage, we do not assume valid values in the
  // context.
  Context<T>* context = this->get_mutable_context();

  // Evaluate f(t,x).
  // TODO: consider taking f(t, x) as argument in case it was already computed
  // somewhere else, as in the NR residual. That'd save one function evaluation
  // per NR iteration.
  context->SetTimeAndContinuousState(t, x);
  const VectorX<T> f = this->EvalTimeDerivatives(*context).CopyToVector();

  // Compute the Jacobian.
  VectorX<T> x_eps = x;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xt| is large, the increment will
    // be large as well. If |xt| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(x(i));
    T dxi = max(1.0, abs_xi) * eps;    

    // Update xt', minimizing the effect of roundoff error by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    x_eps(i) = x(i) + dxi;
    dxi = x_eps(i) - x(i);

    // TODO(sherm1) This is invalidating q, v, and z but we only changed one.
    //              Switch to a method that invalides just the relevant
    //              partition, and ideally modify only the one changed element.
    // Compute f' and set the relevant column of the Jacobian matrix.
    context->SetTimeAndContinuousState(t, x_eps);
    J->col(i) = (this->EvalTimeDerivatives(*context).CopyToVector() - f) / dxi;

    // Reset xt' to xt.
    x_eps(i) = x(i);
  }
}

template <typename T>
bool FixedStepImplicitEulerIntegrator<T>::IsUpdateZero(
    const VectorX<T>& xc, const VectorX<T>& dxc, double eps) const {
  using std::abs;
  using std::max;

  // Reset the tolerance, if necessary, by backing off slightly from the
  // tightest tolerance (machine epsilon).
  if (eps <= 0) eps = 10 * std::numeric_limits<double>::epsilon();

  for (int i = 0; i < xc.size(); ++i) {
    // We do not want the presence of a NaN to cause this function to
    // spuriously return `true`, so indicate the update is not zero when a NaN
    // is detected. This will make the Newton-Raphson process in the caller
    // continue iterating until its inevitable failure.
    using std::isnan;
    if (isnan(dxc[i]) || isnan(xc[i])) return false;

    const T tol = max(abs(xc[i]), T(1)) * eps;
    if (abs(dxc[i]) > tol) return false;
  }

  return true;
}

template <typename T>
bool FixedStepImplicitEulerIntegrator<T>::CheckConvergenceOnStateUpdate(
    const VectorX<T>& xc, const VectorX<T>& dxc, double abs_tol,
    double rel_tol) const {
  using std::abs;
  using std::max;

  for (int i = 0; i < xc.size(); ++i) {
    // We do not want the presence of a NaN to cause this function to
    // spuriously return `true`, so indicate the update is not zero when a NaN
    // is detected. This will make the Newton-Raphson process in the caller
    // continue iterating until its inevitable failure.
    using std::isnan;
    if (isnan(dxc[i]) || isnan(xc[i])) return false;

    const T tol = abs_tol + rel_tol * abs(xc[i]);
    if (abs(dxc[i]) > tol) return false;
  }

  return true;
}

template <typename T>
typename FixedStepImplicitEulerIntegrator<T>::ConvergenceStatus
FixedStepImplicitEulerIntegrator<T>::CheckNewtonConvergence(
    int iteration, const VectorX<T>& x, const VectorX<T>& dx, const T& dx_norm,
    const T& last_dx_norm) const {
  (void)iteration;
  (void)dx_norm;
  (void)last_dx_norm;
  std::cout << "CheckNewtonConvergence()\n";
  PRINT_VAR(iteration);
  PRINT_VAR(dx_norm);
  PRINT_VAR(last_dx_norm);

  // A guess. Ideally we'd expose this or probably better, ask the system for
  // scales.
  const double abs_tol = 1.0e-10;
  // We ask for a tighter convegence on the NR than the accuracy of the
  // integrator.
  const double rel_tol = 0.1 * this->get_accuracy_in_use();
  if (CheckConvergenceOnStateUpdate(x, dx, abs_tol, rel_tol)) {
    return ConvergenceStatus::kConverged;
  } else {
    return ConvergenceStatus::kNotConverged;
  }

  // The check below looks for convergence by identifying cases where the
  // update to the state results in no change.
  // Note: Since we are performing this check at the end of the iteration,
  // after x has been updated, we also know that there is at least some
  // change to the state, no matter how small, on a non-stationary system.
  // Future maintainers should make sure this check only occurs after a change
  // has been made to the state.
  if (IsUpdateZero(x, dx)) {
    std::cout << fmt::format(
        "magnitude of state update indicates convergence\n");
    DRAKE_LOGGER_DEBUG("magnitude of state update indicates convergence");
    return ConvergenceStatus::kConverged;
  }

  // Compute the convergence rate and check convergence.
  // [Hairer, 1996] notes that this convergence strategy should only be applied
  // after *at least* two iterations (p. 121). In practice, we find that it
  // needs to run at least three iterations otherwise some error-controlled runs
  // may choke, hence we check if iteration > 1.
  if (iteration > 1) {
    // TODO(edrumwri) Hairer's RADAU5 implementation (allegedly) uses
    // theta = sqrt(dx[k] / dx[k-2]) while DASSL uses
    // theta = pow(dx[k] / dx[0], 1/k), so investigate setting
    // theta to these alternative values for minimizing convergence failures.
    const T theta = dx_norm / last_dx_norm;
    const T eta = theta / (1 - theta);
    std::cout << fmt::format("Newton-Raphson loop {} theta: {}, eta: {}\n",
                             iteration, theta, eta);
    DRAKE_LOGGER_DEBUG("Newton-Raphson loop {} theta: {}, eta: {}", iteration,
                       theta, eta);

    // Look for divergence.
    if (theta > 1) {
      std::cout << fmt::format("Newton-Raphson divergence detected\n");
      DRAKE_LOGGER_DEBUG("Newton-Raphson divergence detected");
      return ConvergenceStatus::kDiverged;
    }

    // Look for convergence using Equation IV.8.10 from [Hairer, 1996].
    // [Hairer, 1996] determined values of kappa in [0.01, 0.1] work most
    // efficiently on a number of test problems with *Radau5* (a fifth order
    // implicit integrator), p. 121. We select a value halfway in-between.
    const double kappa = 0.05;
    const double k_dot_tol = kappa * this->get_accuracy_in_use();
    if (eta * dx_norm < k_dot_tol) {
      std::cout << fmt::format("Newton-Raphson converged; η = {}\n", eta);
      DRAKE_LOGGER_DEBUG("Newton-Raphson converged; η = {}", eta);
      return ConvergenceStatus::kConverged;
    }
  }

  return ConvergenceStatus::kNotConverged;
}

template <class T>
bool FixedStepImplicitEulerIntegrator<T>::DoStep(const T& h) {
  const int max_newton_iterations = 10;

  // Copy state at t = t0 before we mutate the context.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  const VectorX<T> x0 = context->get_continuous_state().CopyToVector();

  // N.B. calc_residual mutates the base class context, trashing any cached
  // computations.
  const T t = t0 + h;
  auto calc_residual = [t0, h, &x0, context, this](const VectorX<T>& x) {
    context->SetTimeAndContinuousState(t0 + h, x);
    return (x - x0 - h * this->EvalTimeDerivatives(*context).CopyToVector())
        .eval();
  };

  // Initial guess.
  VectorX<T> x_plus = x0;

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  T last_dx_norm = std::numeric_limits<double>::infinity();

  for (int i = 0; i < max_newton_iterations; ++i) {
    ++statistics_.num_nr_iterations;

    const VectorX<T> r = calc_residual(x_plus);

    // Calc Jacobian. For now, full Newton.
    {
      // TODO: Consider using the last computed r(x) if using forward
      // differences. The gain might be negligible.
      CalcJacobian(t, x_plus, &J_);
      ++statistics_.num_jacobian_reforms;
      const int n = J_.rows();
      iteration_matrix_.SetAndFactor(J_ * -h + MatrixX<T>::Identity(n, n));
      ++statistics_.num_factorizations;      
    }

    VectorX<T> dx = iteration_matrix_.Solve(-r);
    // N.B. Integrators use IntegratorBase::CalcStateChangeNorm(). Which
    // "scales" and uses the max norm instead.
    // Study whether that's an appropriate choice or not.
    const T dx_norm = dx.norm();

    x_plus += dx;

    // Check for Newton-Raphson convergence.
    const ConvergenceStatus status =
        CheckNewtonConvergence(i, x_plus, dx, dx_norm, last_dx_norm);
    // If it converged, we're done.
    if (status == ConvergenceStatus::kConverged) {
      context->SetTimeAndContinuousState(t, x_plus);
      return true;
    }
    // If it diverged, we have to abort and try again.
    if (status == ConvergenceStatus::kDiverged) break;
    // Otherwise, continue to the next Newton-Raphson iteration.
    DRAKE_DEMAND(status == ConvergenceStatus::kNotConverged);

    // Update the norm of the state update.
    last_dx_norm = dx_norm;
  }

  DRAKE_LOGGER_DEBUG("DoStep() convergence failed");

  // In full Newton mode, nothing else to do. Report failure so that the
  // simulator can decided whether to reattempt with a smaller time step.
  return false;
}


}  // namespace systems
}  // namespace drake

// N.B. The LU factorization in IterationMatrix does not compile when T =
// AutoDiff for the integraor.
//DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    //class drake::systems::FixedStepImplicitEulerIntegrator)
template class drake::systems::FixedStepImplicitEulerIntegrator<double>;
