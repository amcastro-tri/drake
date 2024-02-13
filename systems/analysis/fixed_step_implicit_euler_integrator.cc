#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/math/autodiff_gradient.h"
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
void FixedStepImplicitEulerIntegrator<T>::DoInitialize() {
  frozen_context_ = this->get_system().CreateDefaultContext();
  derivatives_ = this->get_system().AllocateTimeDerivatives();
  dx_state_ = this->get_system().AllocateTimeDerivatives();
}

template <class T>
void FixedStepImplicitEulerIntegrator<T>::DoResetStatistics() {
  statistics_ = {};
}

template <class T>
void FixedStepImplicitEulerIntegrator<T>::DoPrintStatistics() const {
  fmt::print("FixedStepImplicitEulerIntegrator stats:\n");
  fmt::print("Num DoSteps  = {}\n", statistics_.num_do_steps);
  fmt::print("       h_min = {}\n", statistics_.h_min);
  fmt::print("       h_max = {}\n", statistics_.h_max);
  fmt::print("  num srinks = {}\n", num_step_shrinkages_);  
  fmt::print("Num NR iters = {}\n", statistics_.num_nr_iterations);
  fmt::print("Num derivative evals (residual) = {}\n",
             statistics_.num_function_evaluations);
  fmt::print("Num derivative evals (Jacobian) = {}\n",
             statistics_.num_jacobian_function_evaluations);             
  fmt::print("Num Jacobian updates = {}\n", statistics_.num_jacobian_reforms);
  fmt::print("Num factorizations = {}\n", statistics_.num_factorizations);  
}

template <class T>
void FixedStepImplicitEulerIntegrator<T>::CalcJacobian(const T& t,
    const VectorX<T>& x, MatrixX<T>* J) {
  INSTRUMENT_FUNCTION("Jacobian compuation.");    

  // Get the current number of ODE evaluations.
  int64_t current_derivative_evals =
      use_approximate_derivatives_ ? statistics_.num_function_evaluations
                                   : this->get_num_derivative_evaluations();

  switch (jacobian_scheme_) {
    case JacobianComputationScheme::kForwardDifference:
      ComputeForwardDiffJacobian(t, x, J);
      break;

    case JacobianComputationScheme::kCentralDifference:
      ComputeCentralDiffJacobian(t, x, J);
      break;

    case JacobianComputationScheme::kAutomatic:
      ComputeAutoDiffJacobian(t, x, J);
      break;
  }

  // Update statistics.
  ++statistics_.num_jacobian_reforms;

  // Use the new number of ODE evaluations to determine the number of Jacobian
  // evaluations.
  if (use_approximate_derivatives_) {
    statistics_.num_jacobian_function_evaluations +=
        (statistics_.num_function_evaluations - current_derivative_evals);
  } else {
    statistics_.num_jacobian_function_evaluations +=
        (this->get_num_derivative_evaluations() - current_derivative_evals);
  }
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
      "  computing from state {}", fmt_eigen(x.transpose()));

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
  VectorX<T> f(x.size());
  if (use_approximate_derivatives_) {
    statistics_.num_function_evaluations++;
    this->get_system().CalcApproximateTimeDerivatives(
        *context, *frozen_context_, derivatives_.get());
    f = derivatives_->CopyToVector();
  } else {
    f = this->EvalTimeDerivatives(*context).CopyToVector();
  }

  // Compute the Jacobian.
  VectorX<T> x_eps = x;
  VectorX<T> f_eps(x.size());
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
    if (use_approximate_derivatives_) {
      statistics_.num_function_evaluations++;
      this->get_system().CalcApproximateTimeDerivatives(
          *context, *frozen_context_, derivatives_.get());
      f_eps = derivatives_->CopyToVector();
    } else {
      f_eps = this->EvalTimeDerivatives(*context).CopyToVector();
    }
    J->col(i) = (f_eps - f) / dxi;

    // Reset xt' to xt.
    x_eps(i) = x(i);
  }
}

template <class T>
void FixedStepImplicitEulerIntegrator<T>::ComputeCentralDiffJacobian(
    const T& t, const VectorX<T>& x, MatrixX<T>* J) {
  using std::abs;
  using std::max;

  // Cube root of machine precision (indicated by theory) seems a bit coarse.
  // Pick power of eps halfway between 6/12 (i.e., 1/2) and 4/12 (i.e., 1/3).
  const double eps = std::pow(std::numeric_limits<double>::epsilon(), 5.0 / 12);

  // Get the number of continuous state variables.
  const int n = x.size();

  DRAKE_LOGGER_DEBUG(
      "  FixedStepImplicitEulerIntegrator Compute Centraldiff {}-Jacobian t={}",
      n, t);

  // Initialize the Jacobian.
  J->resize(n, n);

  // scratch context to make system evaluations.
  Context<T>* context = this->get_mutable_context();

  // Set time.
  context->SetTimeAndContinuousState(t, x);

  // Compute the Jacobian.
  VectorX<T> xt_prime = x;
  for (int i = 0; i < n; ++i) {
    // Compute a good increment to the dimension using approximately 1/eps
    // digits of precision. Note that if |xt| is large, the increment will
    // be large as well. If |xt| is small, the increment will be no smaller
    // than eps.
    const T abs_xi = abs(x(i));
    T dxi = max(1.0, abs_xi) * eps;

    // Update xt', minimizing the effect of roundoff error, by ensuring that
    // x and dx differ by an exactly representable number. See p. 192 of
    // Press, W., Teukolsky, S., Vetterling, W., and Flannery, P. Numerical
    //   Recipes in C++, 2nd Ed., Cambridge University Press, 2002.
    xt_prime(i) = x(i) + dxi;
    const T dxi_plus = xt_prime(i) - x(i);

    // TODO(sherm1) This is invalidating q, v, and z but we only changed one.
    //              Switch to a method that invalides just the relevant
    //              partition, and ideally modify only the one changed element.
    // Compute f(x+dx).
    context->SetContinuousState(xt_prime);
    VectorX<T> fprime_plus = this->EvalTimeDerivatives(*context).CopyToVector();

    // Update xt' again, minimizing the effect of roundoff error.
    xt_prime(i) = x(i) - dxi;
    const T dxi_minus = x(i) - xt_prime(i);

    // Compute f(x-dx).
    context->SetContinuousState(xt_prime);
    VectorX<T> fprime_minus =
        this->EvalTimeDerivatives(*context).CopyToVector();

    // Set the Jacobian column.
    J->col(i) = (fprime_plus - fprime_minus) / (dxi_plus + dxi_minus);

    // Reset xt' to xt.
    xt_prime(i) = x(i);
  }
}

template <class T>
void FixedStepImplicitEulerIntegrator<T>::ComputeAutoDiffJacobian(
    const T& t, const VectorX<T>& x, MatrixX<T>* J) {
  DRAKE_LOGGER_DEBUG("  ImplicitIntegrator Compute Autodiff Jacobian t={}", t);
  // TODO(antequ): Investigate how to refactor this method to use
  // math::jacobian(), if possible.

  // Create AutoDiff versions of the state vector.
  // Set the size of the derivatives and prepare for Jacobian calculation.
  VectorX<AutoDiffXd> a_xt = math::InitializeAutoDiff(x);

  const System<T>& system = this->get_system();
  const Context<T>& context = this->get_context();

  // Get the system and the context in AutoDiffable format. Inputs must also
  // be copied to the context used by the AutoDiff'd system (which is
  // accomplished using FixInputPortsFrom()).
  // TODO(edrumwri): Investigate means for moving as many of the operations
  //                 below offline (or with lower frequency than once-per-
  //                 Jacobian calculation) as is possible. These operations
  //                 are likely to be expensive.
  const auto adiff_system = system.ToAutoDiffXd();
  std::unique_ptr<Context<AutoDiffXd>> adiff_context =
      adiff_system->AllocateContext();
  adiff_context->SetTimeStateAndParametersFrom(context);
  adiff_system->FixInputPortsFrom(system, context, adiff_context.get());
  adiff_context->SetTime(t);

  // Set the continuous state in the context.
  adiff_context->SetContinuousState(a_xt);

  // Evaluate the derivatives at that state.
  const VectorX<AutoDiffXd> result =
      this->EvalTimeDerivatives(*adiff_system, *adiff_context).CopyToVector();

  *J = math::ExtractGradient(result);

  // Sometimes the system's derivatives f(t, x) do not depend on its states, for
  // example, when f(t, x) = constant or when f(t, x) depends only on t. In this
  // case, make sure that the Jacobian isn't a n ✕ 0 matrix (this will cause a
  // segfault when forming Newton iteration matrices); if it is, we set it equal
  // to an n x n zero matrix.
  if (J->cols() == 0) {
    *J = MatrixX<T>::Zero(x.size(), x.size());
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
  //std::cout << fmt::format("abs_tol, rel_tol: {}, {}\n", abs_tol, rel_tol);

  for (int i = 0; i < xc.size(); ++i) {
    // We do not want the presence of a NaN to cause this function to
    // spuriously return `true`, so indicate the update is not zero when a NaN
    // is detected. This will make the Newton-Raphson process in the caller
    // continue iterating until its inevitable failure.
    using std::isnan;
    if (isnan(dxc[i]) || isnan(xc[i])) {
      //std::cout << "CheckNewtonConvergence(): NaN values. \n";
      return false;
    }

    const T tol = abs_tol + rel_tol * abs(xc[i]);
    //std::cout << fmt::format("tol, dxi, xi: {}, {} , {}\n", tol, abs(dxc[i]),
    //                         abs(xc[i]));
    if (abs(dxc[i]) > tol) {
      //std::cout << "CheckNewtonConvergence(): Did not converge. \n";
      return false;
    }
  }

  //std::cout << "CheckNewtonConvergence(): Converged. \n";

  return true;
}

template <typename T>
typename FixedStepImplicitEulerIntegrator<T>::ConvergenceStatus
FixedStepImplicitEulerIntegrator<T>::CheckNewtonConvergence(
    int iteration, const VectorX<T>& x, const VectorX<T>& dx, const T& dx_norm,
    const T& last_dx_norm) const {
  //(void)iteration;
  //(void)dx_norm;
  //(void)last_dx_norm;
  //std::cout << "CheckNewtonConvergence()\n";
  //PRINT_VAR(iteration);
  //PRINT_VAR(dx_norm);
  //PRINT_VAR(last_dx_norm);

#if 0
  {
    const T theta = dx_norm / last_dx_norm;
    std::cout << fmt::format("iter, x, dx, theta: {}, {}, {}, {}\n", iteration,
                             x.norm(), dx_norm, theta);
  }
#endif

  // Print progress regardless of the check type.
  const T theta = dx_norm / last_dx_norm;
  fmt::print("it {}, theta: {}, dx: {}\n", iteration, theta, dx_norm);

  if (convergence_check_ == ConvergenceCheck::kUseUpdateAsErrorEstimation) {
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
  }
  // Sanity check there are only two valid options.
  DRAKE_DEMAND(convergence_check_ == ConvergenceCheck::kHairerErrorEstimation);

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
    const T eta = theta / (1 - theta);
    //std::cout << fmt::format("Newton-Raphson loop {} theta: {}, eta: {}\n",
    //                         iteration, theta, eta);
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
      //std::cout << fmt::format("Newton-Raphson converged; η = {}\n", eta);
      DRAKE_LOGGER_DEBUG("Newton-Raphson converged; η = {}", eta);
      return ConvergenceStatus::kConverged;
    }
  }

  return ConvergenceStatus::kNotConverged;
}

template <class T>
bool FixedStepImplicitEulerIntegrator<T>::StepRecursive(int trial, const T& h) {
  constexpr int max_newton_iterations = 10;
  using std::min;
  using std::max;

  fmt::print("StepRecursive: {}, {}\n", trial, h);
  statistics_.h_min = min(statistics_.h_min, h);
  statistics_.h_max = max(statistics_.h_max, h);

  // Copy state at t = t0 before we mutate the context.
  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();
  const VectorX<T> x0 = context->get_continuous_state().CopyToVector();

  if (use_approximate_derivatives_ && trial == 1) {
    frozen_context_->SetTimeAndContinuousState(t0 + h, x0);
  }

  // N.B. calc_residual mutates the base class context, trashing any cached
  // computations.
  const T t = t0 + h;
  auto calc_residual = [t0, h, &x0, context, this](const VectorX<T>& x) {
    const int previous_num_evals = this->get_num_derivative_evaluations();
    context->SetTimeAndContinuousState(t0 + h, x);
    VectorX<T> r(x.size());
    if (use_approximate_derivatives_) {
      this->get_system().CalcApproximateTimeDerivatives(
          *context, *this->frozen_context_, derivatives_.get());
      r = x - x0 - h * derivatives_->CopyToVector();
      this->statistics_.num_function_evaluations++;
    } else {
      r = x - x0 - h * this->EvalTimeDerivatives(*context).CopyToVector();
      this->statistics_.num_function_evaluations +=
        (this->get_num_derivative_evaluations() - previous_num_evals);
    }    
    return r;
  };

  // Initial guess.
  VectorX<T> x_plus = x0;

  // Jacobian at {t0, x0}.
  if (!full_newton_) {
    const int n = x0.size();
    // Calc J0 only on the first trial. Reuse in other trials.
    if (trial == 1) CalcJacobian(t0, x0, &J_);
    iteration_matrix_.SetAndFactor(J_ * -h + MatrixX<T>::Identity(n, n));
    ++statistics_.num_factorizations;
  }

  // Initialize the "last" state update norm; this will be used to detect
  // convergence.
  T last_dx_norm = std::numeric_limits<double>::infinity();

  for (int i = 0; i < max_newton_iterations; ++i) {
    ++statistics_.num_nr_iterations;

    const VectorX<T> r = calc_residual(x_plus);

    // Calc jacobian. Either only once on the first iteration, every iteration.
    if (full_newton_) {
      // TODO: Consider using the last computed r(x) if using forward
      // differences. The gain might be negligible.
      CalcJacobian(t, x_plus, &J_);
      const int n = J_.rows();
      iteration_matrix_.SetAndFactor(J_ * -h + MatrixX<T>::Identity(n, n));
      ++statistics_.num_factorizations;
    }

    VectorX<T> dx = iteration_matrix_.Solve(-r);
    // N.B. Integrators use IntegratorBase::CalcStateChangeNorm(). Which
    // "scales" and uses the max norm instead.
    // Study whether that's an appropriate choice or not.
    // TODO: try different norm options. Only used for convergence checks.
    T dx_norm = NAN;
    switch (error_norm_) {
      case ErrorNorm::kRms:
        dx_norm = dx.norm() / sqrt(dx.size());
        break;
      case ErrorNorm::kMax:
        dx_norm = dx.template lpNorm<Eigen::Infinity>();
        break;
      case ErrorNorm::kWeighted:
        dx_state_->get_mutable_vector().SetFromVector(dx);
        dx_norm = this->CalcStateChangeNorm(*dx_state_);
        break;
    }

    x_plus += dx;

    // Check for Newton-Raphson convergence.
    const ConvergenceStatus status =
        CheckNewtonConvergence(i, x_plus, dx, dx_norm, last_dx_norm);
    // If it converged, we're done.
    if (status == ConvergenceStatus::kConverged) {
      fmt::print("Converged. t: {}, h: {}\n", t, h);
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

  DRAKE_LOGGER_DEBUG("StepRecursive() convergence failed");

  // If it failed, retry a smaller time step until we succeed.
  // Reset state to {t0, x0} for the next smaller step.
  context->SetTimeAndContinuousState(t0, x0);
  last_step_was_adjusted_ = true;
  num_step_shrinkages_++;
  return StepRecursive(trial + 1, h / 2.0);
}

template <class T>
bool FixedStepImplicitEulerIntegrator<T>::DoStep(const T& h) {
  INSTRUMENT_FUNCTION("Integrator main entry point.");
  using std::min;
  using std::max;

  // Reset. We always try h first.
  last_step_was_adjusted_ = false;

  ++statistics_.num_do_steps;  

  fmt::print("\n");
  fmt::print("DoStep: {}\n", statistics_.num_do_steps);    

  Context<T>* context = this->get_mutable_context();
  const T t0 = context->get_time();

  fmt::print(" t0: {}, h: {}\n", t0, h);  

  // This recursive search of a successfull step must always succeed.
  bool success = StepRecursive(1, h);
  DRAKE_DEMAND(success);
  return success;

#if 0
  // DoStep()'s contract is to return the state to {t0, x0} if it fails!
  // TODO: Make NVI Step() do this instead.
  context->SetTimeAndContinuousState(t0, x0);

  // In full Newton mode, nothing else to do. Report failure so that the
  // simulator can decided whether to reattempt with a smaller time step.
  return false;
#endif  
}


}  // namespace systems
}  // namespace drake

// N.B. The LU factorization in IterationMatrix does not compile when T =
// AutoDiff for the integraor.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::FixedStepImplicitEulerIntegrator)
//template class drake::systems::FixedStepImplicitEulerIntegrator<double>;
