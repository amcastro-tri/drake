#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace systems {

/**
 * A first-order, implicit Euler integrator. State is updated in the following
 * manner:
 * <pre>
 * x(t+h) = x(t) + h * dx/dt(t, x(t))
 * </pre>
 *
 * @tparam_default_scalar
 * @ingroup integrators
 */
template <class T>
class FixedStepImplicitEulerIntegrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedStepImplicitEulerIntegrator)

  class IterationMatrix {
   public:
    /// Factors a dense matrix (the iteration matrix) using LU factorization,
    /// which should be faster than the QR factorization used in the specialized
    /// template method for AutoDiffXd below.
    void SetAndFactor(const MatrixX<T>& iteration_matrix);

    /// Solves a linear system Ax = b for x using the iteration matrix (A)
    /// factored using LU decomposition.
    /// @see Factor()
    VectorX<T> Solve(const VectorX<T>& b) const;

    /// Returns whether the iteration matrix has been set and factored.
    bool matrix_factored() const { return matrix_factored_; }

   private:
    bool matrix_factored_{false};

    // A simple LU factorization is all that is needed for ImplicitIntegrator
    // templated on scalar type `double`; robustness in the solve
    // comes naturally as h << 1. Keeping this data in the class definition
    // serves to minimize heap allocations and deallocations.
    Eigen::PartialPivLU<MatrixX<double>> LU_;

    // The only factorization supported by automatic differentiation in Eigen is
    // currently QR. When ImplicitIntegrator is templated on type AutoDiffXd,
    // this will be the factorization that is used.
    Eigen::HouseholderQR<MatrixX<AutoDiffXd>> QR_;
  };

  ~FixedStepImplicitEulerIntegrator() override = default;

  /** Constructs a fixed-step integrator. */
  FixedStepImplicitEulerIntegrator(const System<T>& system,
                                   const T& max_step_size,
                                   Context<T>* context = nullptr)
      : IntegratorBase<T>(system, context) {
    IntegratorBase<T>::set_maximum_step_size(max_step_size);
  }

  enum class JacobianComputationScheme {
    /// Forward differencing.
    kForwardDifference,

    /// Central differencing.
    kCentralDifference,

    /// Automatic differentiation.
    kAutomatic
  };

  enum class ConvergenceCheck {
    /// Given the update Δxᵏ = xᵏ−xᵏ⁻¹ at the k-th iteration, estimates the
    /// error to the converged solution x* as:
    ///   ‖x* − xᵏ‖ ≤ θ/(1−θ)⋅‖Δxᵏ‖
    /// See [Hairer, 1996] notes on page 121.
    kHairerErrorEstimation,

    /// Uses the norm ‖Δxᵏ‖ directly for convergence check.
    kUseUpdateAsErrorEstimation,
  };

  // The norm used for error estimations.
  enum class ErrorNorm {
    // Uses rms norm: e = ‖x‖/√{n}.
    kRms,

    // Uses max norm: e = ‖x‖∞.
    kMax,

    // Uses weighted norm according to CalcStateChangeNorm().
    kWeighted,
  };

  void set_convergence_check(ConvergenceCheck type) {
    convergence_check_ = type;
  }

  void set_error_norm(ErrorNorm type) {
    error_norm_ = type;
  }

  void set_full_newton(bool use_full_newton) { full_newton_ = use_full_newton; }

  void use_approximate_derivatives(bool use_approximate) {
    use_approximate_derivatives_ = use_approximate;
  }

  /**
   * This integrator does not support error estimation.
   */
  bool supports_error_estimation() const final { return false; }

  /** Integrator does not provide an error estimate. */
  int get_error_estimate_order() const final { return 0; }

  /// Sets the Jacobian computation scheme. This function can be safely called
  /// at any time (i.e., the integrator need not be re-initialized afterward).
  /// @note Discards any already-computed Jacobian matrices if the scheme
  ///       changes.
  void set_jacobian_computation_scheme(JacobianComputationScheme scheme) {
    if (jacobian_scheme_ != scheme) {
      J_.resize(0, 0);
      iteration_matrix_ = {};
    }
    jacobian_scheme_ = scheme;
  }

  JacobianComputationScheme get_jacobian_computation_scheme() const {
    return jacobian_scheme_;
  }

 private:
  enum class ConvergenceStatus {
    kDiverged,
    kConverged,
    kNotConverged,
  };

  // These are statistics that the base class, ImplicitIntegrator, require
  // this child class to keep track of.
  struct Statistics {
    int64_t num_do_steps{0};

    // Min step size DoStep() was called with.
    T h_min{std::numeric_limits<double>::max()};

    // Max step size DoStep() was called with.
    T h_max{0.0};  // Max step sized DoStep() was called with.

    // See ImplicitIntegrator::get_num_newton_raphson_iterations()
    // or ImplicitIntegrator::
    // get_num_error_estimator_newton_raphson_iterations() for the definition
    // of this statistic.
    int64_t num_nr_iterations{0};

    // Function evaluations for residual only.
    int64_t num_function_evaluations{0};

    // See ImplicitIntegrator::get_num_jacobian_evaluations() or
    // ImplicitIntegrator::get_num_error_estimator_jacobian_evaluations()
    // for the definition of this statistic.
    int64_t num_jacobian_reforms{0};

    // See ImplicitIntegrator::get_num_iteration_matrix_factorizations() or
    // ImplicitIntegrator::
    // get_num_error_estimator_iteration_matrix_factorizations() for the
    // definition of this statistic.
    int64_t num_factorizations{0};

    // See ImplicitIntegrator::get_num_derivative_evaluations_for_jacobian()
    // or ImplicitIntegrator::
    // get_num_error_estimator_derivative_evaluations_for_jacobian()
    // for the definition of this statistic.
    int64_t num_jacobian_function_evaluations{0};    
  };  

  // IntegratorBase implementations.
  void DoInitialize() final;
  void DoResetStatistics() final;
  bool DoStep(const T& h) final;
  void DoPrintStatistics() const final;
  bool last_step_was_adjusted() const final {
    return last_step_was_adjusted_;
  }

  bool StepRecursive(int trial, const T& h);

  // non-const because updates stats.
  void CalcJacobian(const T& t, const VectorX<T>& x, MatrixX<T>* J);

  // non-const because it uses the integrator's context as scratch.
  void ComputeForwardDiffJacobian(const T& t, const VectorX<T>& x,
                                  MatrixX<T>* J);
  void ComputeCentralDiffJacobian(const T& t, const VectorX<T>& x,
                                  MatrixX<T>* J);
  void ComputeAutoDiffJacobian(const T& t, const VectorX<T>& x, MatrixX<T>* J);

  bool IsUpdateZero(const VectorX<T>& xc, const VectorX<T>& dxc,
                    double eps = -1.0) const;

  bool CheckConvergenceOnStateUpdate(const VectorX<T>& xc,
                                     const VectorX<T>& dxc, double abs_tol,
                                     double rel_tol) const;

  ConvergenceStatus CheckNewtonConvergence(int iteration, const VectorX<T>& x,
                                           const VectorX<T>& dx,
                                           const T& dx_norm,
                                           const T& last_dx_norm) const;

  JacobianComputationScheme jacobian_scheme_{
      JacobianComputationScheme::kForwardDifference};

  // The last computed Jacobian matrix.
  MatrixX<T> J_;      

  // The last computed iteration matrix and factorization.
  IterationMatrix iteration_matrix_;

  bool full_newton_{true};

  bool last_step_was_adjusted_{false};

  bool use_approximate_derivatives_{false};

  int num_step_shrinkages_{0};

  std::unique_ptr<Context<T>> frozen_context_;
  std::unique_ptr<ContinuousState<T>> derivatives_;

  // The continuous state update vector used during Newton-Raphson.
  // Only used when error_norm_ = ErrorNorm::kWeighted.
  std::unique_ptr<ContinuousState<T>> dx_state_;

  // Statistics for the Newton-Raphson solve.
  Statistics statistics_;

  ConvergenceCheck convergence_check_{
      ConvergenceCheck::kUseUpdateAsErrorEstimation};

  ErrorNorm error_norm_{ErrorNorm::kRms};
};

template <>
inline void
FixedStepImplicitEulerIntegrator<AutoDiffXd>::ComputeAutoDiffJacobian(
    const AutoDiffXd&, const VectorX<AutoDiffXd>&, MatrixX<AutoDiffXd>*) {
  throw std::runtime_error(
      "AutoDiff'd Jacobian not supported from "
      "AutoDiff'd ImplicitIntegrator");
}

template <>
inline void
FixedStepImplicitEulerIntegrator<AutoDiffXd>::IterationMatrix::SetAndFactor(
    const MatrixX<AutoDiffXd>& iteration_matrix) {
  QR_.compute(iteration_matrix);
  matrix_factored_ = true;
}

template <>
inline VectorX<AutoDiffXd>
FixedStepImplicitEulerIntegrator<AutoDiffXd>::IterationMatrix::Solve(
    const VectorX<AutoDiffXd>& b) const {
  return QR_.solve(b);
}

}  // namespace systems
}  // namespace drake

// N.B. The LU factorization in IterationMatrix does not compile when T =
// AutoDiff for the integraor.
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::FixedStepImplicitEulerIntegrator)
//extern template class drake::systems::FixedStepImplicitEulerIntegrator<double>;
