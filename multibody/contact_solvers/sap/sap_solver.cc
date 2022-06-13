#include "drake/multibody/contact_solvers/sap/sap_solver.h"

#include <algorithm>
#include <limits>
#include <type_traits>
#include <deque>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/extract_double.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/linear_solve.h"
#include "drake/multibody/contact_solvers/supernodal_solver.h"
#include "drake/multibody/contact_solvers/newton_with_bisection.h"
#include "drake/multibody/plant/stop_watch.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
//#define PRINT_VAR(a) (void)a;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using drake::systems::Context;
using drake::test::LimitMalloc;
using drake::multibody::internal::StopWatch;

template <typename T>
void SapSolver<T>::set_parameters(const SapSolverParameters& parameters) {
  parameters_ = parameters;
}

template <typename T>
const typename SapSolver<T>::SolverStats& SapSolver<T>::get_statistics() const {
  return stats_;
}

template <typename T>
void SapSolver<T>::PackSapSolverResults(const systems::Context<T>& context,
                                        SapSolverResults<T>* results) const {
  DRAKE_DEMAND(results != nullptr);
  results->Resize(model_->problem().num_velocities(),
                  model_->num_constraint_equations());

  // For non-participating velocities the solutions is v = v*. Therefore we
  // first initialize to v = v* and overwrite with the non-trivial participating
  // values in the following line.
  results->v = model_->problem().v_star();
  const VectorX<T>& v_participating = model_->GetVelocities(context);
  model_->velocities_permutation().ApplyInverse(v_participating, &results->v);

  // Constraints equations are clustered (essentially their order is permuted
  // for a better sparsity structure). Therefore constraint velocities and
  // impulses are evaluated in this clustered order and permuted into the
  // original order described by the model right after.
  const VectorX<T>& vc_clustered = model_->EvalConstraintVelocities(context);
  model_->impulses_permutation().ApplyInverse(vc_clustered, &results->vc);
  const VectorX<T>& gamma_clustered = model_->EvalImpulses(context);
  model_->impulses_permutation().ApplyInverse(gamma_clustered, &results->gamma);

  // For non-participating velocities we have v=v* and the generalized impulses
  // are zero. Therefore we first zero-out all generalized impulses and
  // overwrite with the non-trivial non-zero values for the participating DOFs
  // right after.
  const VectorX<T>& tau_participating =
      model_->EvalGeneralizedImpulses(context);
  results->j.setZero();
  model_->velocities_permutation().ApplyInverse(tau_participating, &results->j);
}

template <typename T>
void SapSolver<T>::CalcStoppingCriteriaResidual(const Context<T>& context,
                                                T* momentum_residual,
                                                T* momentum_scale) const {
  using std::max;
  const VectorX<T>& inv_sqrt_A = model_->inv_sqrt_dynamics_matrix();
  const VectorX<T>& p = model_->EvalMomentum(context);
  const VectorX<T>& jc = model_->EvalGeneralizedImpulses(context);
  const VectorX<T>& ell_grad = model_->EvalCostGradient(context);

  // Scale generalized momentum quantities using inv_sqrt_A so that all entries
  // have the same units and we can weigh them equally.
  const VectorX<T> ell_grad_tilde = inv_sqrt_A.asDiagonal() * ell_grad;
  const VectorX<T> p_tilde = inv_sqrt_A.asDiagonal() * p;
  const VectorX<T> jc_tilde = inv_sqrt_A.asDiagonal() * jc;

  *momentum_residual = ell_grad_tilde.norm();
  *momentum_scale = max(p_tilde.norm(), jc_tilde.norm());
}

template <typename T>
SapSolverStatus SapSolver<T>::SolveWithGuess(const SapContactProblem<T>&,
                                             const VectorX<T>&,
                                             SapSolverResults<T>*) {
  throw std::logic_error(
      "SapSolver::SolveWithGuess(): Only T = double is supported.");
}

template <>
SapSolverStatus SapSolver<double>::SolveWithGuess(
    const SapContactProblem<double>& problem, const VectorX<double>& v_guess,
    SapSolverResults<double>* results) {
  using std::abs;
  using std::max;

  if (problem.num_constraints() == 0) {
    // In the absence of constraints the solution is trivially v = v*.
    results->Resize(problem.num_velocities(),
                    problem.num_constraint_equations());
    results->v = problem.v_star();
    results->j.setZero();
    return SapSolverStatus::kSuccess;
  }

  // Make model for the given contact problem.
  model_ = std::make_unique<SapModel<double>>(&problem);
  const int nv = model_->num_velocities();
  const int nk = model_->num_constraint_equations();

  // Allocate the necessary memory to work with.
  auto context = model_->MakeContext();
  auto scratch = model_->MakeContext();
  SearchDirectionData search_direction_data(nv, nk);
  std::deque<double> cost_deque;  // Save last M values for GLL line search.
  stats_ = SolverStats();
  stats_.num_constraints = model_->num_constraints();
  stats_.num_constraint_equations = model_->num_constraint_equations();

  // The supernodal solver is expensive to instantiate and therefore we only
  // instantiate when needed.
  std::unique_ptr<SuperNodalSolver> supernodal_solver;

  {
    // We limit the lifetime of this reference, v, to within this scope where we
    // immediately need it.
    Eigen::VectorBlock<VectorX<double>> v =
        model_->GetMutableVelocities(context.get());
    model_->velocities_permutation().Apply(v_guess, &v);
  }

  // Start Newton iterations.
  int k = 0;
  double ell_previous = model_->EvalCost(*context);
  cost_deque.push_front(ell_previous);
  double cost_max = ell_previous;
  double cost_max_previous = cost_max;
  bool converged = false;
  for (;; ++k) {
    // We first verify the stopping criteria. If satisfied, we skip expensive
    // factorizations.
    double momentum_residual, momentum_scale;
    CalcStoppingCriteriaResidual(*context, &momentum_residual, &momentum_scale);
    stats_.optimality_criterion_reached =
        momentum_residual <=
        parameters_.abs_tolerance + parameters_.rel_tolerance * momentum_scale;
    stats_.momentum_residual.push_back(momentum_residual);
    stats_.momentum_scale.push_back(momentum_scale);
    // TODO(amcastro-tri): consider monitoring the duality gap.
    if (stats_.optimality_criterion_reached || stats_.cost_criterion_reached) {
      converged = true;
      break;
    } else {
      if (!parameters_.use_dense_algebra && supernodal_solver == nullptr) {
        // Instantiate supernodal solver on the first iteration when needed. If
        // the stopping criteria is satisfied at k = 0 (good guess), then we
        // skip the expensive instantiation of the solver.
        StopWatch watch;
        supernodal_solver = MakeSuperNodalSolver();
        stats_.make_solver_time = watch.Elapsed();
      }
    }

    // Exit if the maximum number of iterations is reached, but only after
    // checking the convergence criteria, so that also the last iteration is
    // considered.
    if (k == parameters_.max_iterations) break;

    // This is the most expensive update: it performs the factorization of H to
    // solve for the search direction dv.
    CalcSearchDirectionData(*context, supernodal_solver.get(),
                            &search_direction_data);
    const VectorX<double>& dv = search_direction_data.dv;    

    double alpha = 1.0;
    int ls_iters = 0;
    StopWatch watch;
    switch (parameters_.line_search_type) {
      case SapSolverParameters::LineSearchType::kBackTracking:
        std::tie(alpha, ls_iters) = PerformBackTrackingLineSearch(
            *context, search_direction_data, scratch.get());
        break;
      case SapSolverParameters::LineSearchType::kExact:
        std::tie(alpha, ls_iters) = PerformExactLineSearch(
            *context, search_direction_data, scratch.get());
        break;
      case SapSolverParameters::LineSearchType::kGll:
        const bool use_armijo = k < parameters_.gll_num_armijos;
        std::tie(alpha, ls_iters) =
            PerformGllLineSearch(*context, search_direction_data, cost_deque,
                                 use_armijo, scratch.get());
        break;
    }
    stats_.ls_time += watch.Elapsed();

    stats_.num_line_search_iters += ls_iters;

    // Update state.
    model_->GetMutableVelocities(context.get()) += alpha * dv;

    const double ell = model_->EvalCost(*context);

    // At iteration k, the cost queue will always contain ell_k plus all
    // previous costs ell_{k-j}, with j = 0, gll_num_previous_costs.
    cost_deque.push_front(ell);
    if (static_cast<int>(cost_deque.size()) >
        parameters_.gll_num_previous_costs + 1)
      cost_deque.pop_back();
    cost_max_previous = cost_max;      
    cost_max = *std::max_element(cost_deque.begin(), cost_deque.end());      

    // Sanity check size of the costs queue.
    DRAKE_DEMAND(static_cast<int>(cost_deque.size()) <=
                 parameters_.gll_num_previous_costs + 1);

    const double ell_scale = (ell + ell_previous) / 2.0;
    // N.B. Even though theoretically we expect ell < ell_previous, round-off
    // errors might make the difference ell_previous - ell negative, within
    // machine epsilon. Therefore we take the absolute value here.
    const double ell_decrement = std::abs(ell_previous - ell);

    // SAP's convergence is monotonic. We sanity check this here. We use a slop
    // to account for round-off errors.
    // TODO(amcastro-tri): We might need to loosen this slop or make this an
    // ASSERT instead.
    const double slop =
        50 * std::numeric_limits<double>::epsilon() * std::max(1.0, ell_scale);    
    if (parameters_.line_search_type ==
        SapSolverParameters::LineSearchType::kGll) {
      // GLL allows non-monotone convergence. However the sequence {cost_max} is
      // nonincreasing.
      DRAKE_DEMAND(cost_max <= cost_max_previous + slop);          
    } else {            
      // TODO: Most likely I need to move this to be AFER the check for
      // convergence to avoid false positives. Otherwise this might only be
      // triggered because of round-off errors at the minimum.
      if (ell > ell_previous) {
        // Warn to console.
        std::cout << fmt::format("Cost increased by: {}.\n",
                                 std::abs(ell - ell_previous));
        //PRINT_VAR(ell);
        //PRINT_VAR(std::abs(ell-ell_previous));
        //PRINT_VAR(slop);
        //PRINT_VAR(ell_scale);
      }
      //DRAKE_DEMAND(ell <= ell_previous + slop);      
    }

    // N.B. Here we want alpha≈1 and therefore we impose alpha > 0.5, an
    // arbitrarily "large" value. This is to avoid a false positive on the
    // convergence of the cost due to a small value of the line search
    // parameter.
    stats_.cost_criterion_reached =
        ell_decrement < parameters_.cost_abs_tolerance +
                            parameters_.cost_rel_tolerance * ell_scale &&
        alpha > 0.5;

    ell_previous = ell;
  }

  if (!converged) return SapSolverStatus::kFailure;

  PackSapSolverResults(*context, results);

  // N.B. If the stopping criteria is satisfied for k = 0, the solver is not
  // even instantiated and no factorizations are performed (the expensive part
  // of the computation). We report zero number of iterations.
  stats_.num_iters = k;

  return SapSolverStatus::kSuccess;
}

template <typename T>
T SapSolver<T>::CalcCostAlongLine(
    const systems::Context<T>& context,
    const SearchDirectionData& search_direction_data, const T& alpha,
    systems::Context<T>* scratch, 
    VectorX<T>* vec_scratch,
    T* dell_dalpha, T* d2ell_dalpha2) const {
  // If dell_dalpha is requested, then d2ell_dalpha2 must also be requested.
  if (dell_dalpha != nullptr) DRAKE_DEMAND(d2ell_dalpha2 != nullptr);  

  // We expect no allocations beyond this point.
  //LimitMalloc guard({.max_num_allocations = 0});

  // Data.
  const VectorX<T>& R = model_->constraints_bundle().R();
  const VectorX<T>& v_star = model_->v_star();

  // Search direction quantities at state v.
  const VectorX<T>& dv = search_direction_data.dv;
  const VectorX<T>& dp = search_direction_data.dp;
  const VectorX<T>& dvc = search_direction_data.dvc;
  const T& d2ellA_dalpha2 = search_direction_data.d2ellA_dalpha2;

  // State at v(alpha).
  Context<T>& context_alpha = *scratch;
  const VectorX<T>& v = model_->GetVelocities(context);
  model_->GetMutableVelocities(&context_alpha) = v + alpha * dv;

  // Update velocities and impulses at v(alpha).
  const VectorX<T>& gamma = model_->EvalImpulses(context_alpha);

  // Regularizer cost.
  const T ellR = 0.5 * gamma.dot(R.asDiagonal() * gamma);

  // Momentum cost. We use the O(n) strategy described in [Castro et al., 2021].
  // The momentum cost is: ellA(α) = 0.5‖v(α)−v*‖², where ‖⋅‖ is the norm
  // defined by A. v(α) corresponds to the value of v along the search
  // direction: v(α) = v + αΔv. Using v(α) in the expression of the cost and
  // expanding the squared norm leads to: ellA(α) = 0.5‖v−v*‖² + αΔvᵀ⋅A⋅(v−v*) +
  // 0.5‖Δv‖²α². We now notice some of those terms are already cached:
  //  - dpᵀ = Δvᵀ⋅A
  //  - ellA(v) = 0.5‖v−v*‖²
  //  - d2ellA_dalpha2 = 0.5‖Δv‖²α², see [Castro et al., 2021; §VIII.C].
  T ellA = model_->EvalMomentumCost(context);
  ellA += alpha * dp.dot(v - v_star);
  ellA += 0.5 * alpha * alpha * d2ellA_dalpha2;
  const T ell = ellA + ellR;

  // Compute both dell_dalpha & d2ell_dalpha2
  if (dell_dalpha != nullptr) {
    vec_scratch->resize(model_->num_constraint_equations());
    const VectorX<T>& v_alpha = model_->GetVelocities(context_alpha);

    // First derivative.
    const T dellA_dalpha = dp.dot(v_alpha - v_star);  // Momentum term.
    const T dellR_dalpha = -dvc.dot(gamma);     // Regularizer term.
    *dell_dalpha = dellA_dalpha + dellR_dalpha;

    // Second derivative.    
    const std::vector<MatrixX<T>>& G =
        model_->EvalConstraintsHessian(context_alpha);
    const int nc = model_->num_constraints();
    int constraint_start = 0;
    for (int i = 0; i < nc; ++i) {
      const MatrixX<T>& G_i = G[i];
      // Number of equations for the i-th constraint.
      const int ni = G_i.rows();
      const auto dvc_i = dvc.segment(constraint_start, ni);

      vec_scratch->segment(constraint_start, ni) = G_i * dvc_i;

#if 0
      // std::vector does not allocate if ni <= capacity.
      storage.resize(ni);
      Eigen::Map<VectorX<T>> tmp(storage.data(), ni);

      tmp.noalias() = G_i * dvc_i;
      const T d2ellR_dalpha2_ic = dvc_i.dot(tmp);
      // Theoretically we should have d2ellR_dalpha2_ic > 0. We check this
      // conditions modulo a slop to account for round-off errors.
      const double ell0 = ExtractDoubleOrThrow(model_->EvalCost(context));
      const double slop =
          50.0 * std::abs(ell0) * std::numeric_limits<double>::epsilon();
      DRAKE_DEMAND(d2ellR_dalpha2_ic > -slop);

      // clip any machine epsilon number smaller than zero to avoid accumulation
      // of small negative numbers. This is ok since we checked above that
      // d2ellR_dalpha2_ic is positive within a slop tolerance.
      using std::max;
      d2ellR_dalpha2 += max(0.0, d2ellR_dalpha2_ic);
#endif      

      constraint_start += ni;
    }

    // = Δvcᵀ⋅G⋅Δvc
    const T d2ellR_dalpha2 = dvc.dot(*vec_scratch);

    *d2ell_dalpha2 = d2ellA_dalpha2 + d2ellR_dalpha2;

    // Sanity check these terms are all positive.
    DRAKE_DEMAND(d2ellR_dalpha2 >= 0.0);
    DRAKE_DEMAND(d2ellA_dalpha2 > 0.0);
    DRAKE_DEMAND(*d2ell_dalpha2 > 0);
  }

  return ell;
}

template <typename T>
std::pair<T, int> SapSolver<T>::PerformBackTrackingLineSearch(
    const systems::Context<T>& context,
    const SearchDirectionData& search_direction_data,
    systems::Context<T>* scratch) const {
  using std::abs;
  // Line search parameters.
  const double rho = parameters_.ls_rho;
  const double c = parameters_.ls_c;
  const int max_iterations = parameters_.ls_max_iterations;

  // Quantities at alpha = 0.
  const T& ell0 = model_->EvalCost(context);
  const VectorX<T>& ell_grad_v0 = model_->EvalCostGradient(context);

  // dℓ/dα(α = 0) = ∇ᵥℓ(α = 0)⋅Δv.
  const VectorX<T>& dv = search_direction_data.dv;
  const T dell_dalpha0 = ell_grad_v0.dot(dv);

  T alpha = parameters_.ls_alpha_max;
  T ell = CalcCostAlongLine(context, search_direction_data, alpha, scratch);

  const double kTolerance = 50 * std::numeric_limits<double>::epsilon();
  // N.B. We expect ell_scale != 0 since otherwise SAP's optimality condition
  // would've been reached and the solver would not reach this point.
  // N.B. ell = 0 implies v = v* and gamma = 0, for which the momentum residual
  // is zero.
  // Given that the Hessian in SAP is SPD we know that dell_dalpha0 < 0
  // (strictly). dell_dalpha0 = 0 would mean that we reached the optimum but
  // most certainly due to round-off errors or very tight user specified
  // optimality tolerances, the optimality condition was not met and SAP
  // performed an additional iteration to find a search direction that, most
  // likely, is close to zero. We therefore detect this case with dell_dalpha0 ≈
  // 0 and accept the search direction with alpha = 1.
  const T ell_scale = 0.5 * (ell + ell0);
  if (abs(dell_dalpha0 / ell_scale) < kTolerance) return std::make_pair(1.0, 0);

  // dℓ/dα(α = 0) is guaranteed to be strictly negative given the the Hessian of
  // the cost is positive definite. Only round-off errors in the factorization
  // of the Hessian for ill-conditioned systems (small regularization) can
  // destroy this property. If so, we abort given that'd mean the model must be
  // revisited.
  if (dell_dalpha0 >= 0) {
    throw std::runtime_error(
        "The cost does not decrease along the search direction. This is "
        "usually caused by an excessive accumulation round-off errors for "
        "ill-conditioned systems. Consider revisiting your model.");
  }

  // Verifies if ell(alpha) satisfies Armijo's criterion.
  auto satisfies_armijo = [c, ell0, dell_dalpha0](const T& alpha_in,
                                                  const T& ell_in) {
    return ell_in < ell0 + c * alpha_in * dell_dalpha0;
  };

  // Initialize previous iteration values.
  T alpha_prev = alpha;
  T ell_prev = ell;

  int iteration = 1;
  for (; iteration < max_iterations; ++iteration) {
    alpha *= rho;
    ell = CalcCostAlongLine(context, search_direction_data, alpha, scratch);
    // We scan discrete values of alpha from alpha_max to zero and seek for the
    // minimum value of the cost evaluated at those discrete values. Since we
    // know the cost is convex, we detect this minimum by checking the condition
    // ell > ell_prev. If Armijo's criterion is satisfied, we are done.
    // Otherwise we continue iterating until Armijo's criterion is satisfied.
    // N.B. Armijo's criterion allows to prove convergence. Therefore we want
    // the search parameter to satisfy it.
    if (ell > ell_prev && satisfies_armijo(alpha, ell)) {
      // The previous iteration is better if it satisfies Armijo's
      // criterion since in this scope ell_prev < ell. If so, we
      // backtrack to the previous iteration.
      if (satisfies_armijo(alpha_prev, ell_prev)) alpha /= rho;
      return std::make_pair(alpha, iteration);
    }
    alpha_prev = alpha;
    ell_prev = ell;
  }

  // If the very last iterate satisfies Armijo's, we use it.
  if (satisfies_armijo(alpha, ell)) return std::make_pair(alpha, iteration);

  // If we are here, the line-search could not find a valid parameter that
  // satisfies Armijo's criterion.
  throw std::runtime_error(
      "Line search reached the maximum number of iterations. Either we need to "
      "increase the maximum number of iterations parameter or to condition the "
      "problem better.");

  // Silence "no-return value" warning from the compiler.
  DRAKE_UNREACHABLE();
}

template <typename T>
std::pair<T, int> SapSolver<T>::PerformGllLineSearch(
    const systems::Context<T>& context,
    const SearchDirectionData& search_direction_data,
    const std::deque<double>& cost_deque,
    bool use_armijo, 
    systems::Context<T>* scratch) const {
  using std::abs;
  // Line search parameters.
  const double rho = parameters_.ls_rho;
  const double c = parameters_.ls_c;
  const int max_iterations = parameters_.ls_max_iterations;

  // Quantities at alpha = 0.
  const T& ell0 = model_->EvalCost(context);
  const VectorX<T>& ell_grad_v0 = model_->EvalCostGradient(context);

  // dℓ/dα(α = 0) = ∇ᵥℓ(α = 0)⋅Δv.
  const VectorX<T>& dv = search_direction_data.dv;
  const T dell_dalpha0 = ell_grad_v0.dot(dv);

  T alpha = parameters_.ls_alpha_max;
  T dell, d2ell;
  VectorX<T> vec_scratch;
  T ell = CalcCostAlongLine(context, search_direction_data, alpha, scratch,
                            &vec_scratch, &dell, &d2ell);
  // If the cost keeps decreasing at alpha_max, then alpha = alpha_max is a good
  // step size.                        
  if (dell <= 0) return std::make_pair(alpha, 0);

  const double kTolerance = 50 * std::numeric_limits<double>::epsilon();
  // N.B. We expect ell_scale != 0 since otherwise SAP's optimality condition
  // would've been reached and the solver would not reach this point.
  // N.B. ell = 0 implies v = v* and gamma = 0, for which the momentum residual
  // is zero.
  // Given that the Hessian in SAP is SPD we know that dell_dalpha0 < 0
  // (strictly). dell_dalpha0 = 0 would mean that we reached the optimum but
  // most certainly due to round-off errors or very tight user specified
  // optimality tolerances, the optimality condition was not met and SAP
  // performed an additional iteration to find a search direction that, most
  // likely, is close to zero. We therefore detect this case with dell_dalpha0 ≈
  // 0 and accept the search direction with alpha = 1.
  const T ell_scale = 0.5 * (ell + ell0);
  if (abs(dell_dalpha0 / ell_scale) < kTolerance) return std::make_pair(1.0, 0);

  // dℓ/dα(α = 0) is guaranteed to be strictly negative given the the Hessian of
  // the cost is positive definite. Only round-off errors in the factorization
  // of the Hessian for ill-conditioned systems (small regularization) can
  // destroy this property. If so, we abort given that'd mean the model must be
  // revisited.
  if (dell_dalpha0 >= 0) {
    throw std::runtime_error(
        "The cost does not decrease along the search direction. This is "
        "usually caused by an excessive accumulation round-off errors for "
        "ill-conditioned systems. Consider revisiting your model.");
  }  

  // Verifies if ell(alpha) satisfies GLL's criterion.
  // TODO: Verify the sequence {cost_max} is nonincreasing. Eq. (5) in GLL's
  // Theorem.
  const double cost_max =
      use_armijo ? cost_deque.front()
                 : *std::max_element(cost_deque.begin(), cost_deque.end());
  auto satisfies_gll = [c, cost_max, dell_dalpha0](const T& alpha_in,
                                                   const T& ell_in) {
    return ell_in < cost_max + c * alpha_in * dell_dalpha0;
  };

  // Initialize previous iteration values.
  //T alpha_prev = alpha;
  //T ell_prev = ell;

  int iteration = 1;  // We already evaluated cost at ls_alpha_max.
  for (; iteration < max_iterations; ++iteration) {    
    //if (ell > ell_prev && satisfies_gll(alpha, ell)) {
    if (satisfies_gll(alpha, ell)) {      
      return std::make_pair(alpha, iteration);
    }
    alpha *= rho;
    ell = CalcCostAlongLine(context, search_direction_data, alpha, scratch);
    //alpha_prev = alpha;
    //ell_prev = ell;
  }

  // If we are here, the line-search could not find a valid parameter that
  // satisfies GLL's criterion.
  throw std::runtime_error(
      "Line search reached the maximum number of iterations. Either we need to "
      "increase the maximum number of iterations parameter or to condition the "
      "problem better.");

  // Silence "no-return value" warning from the compiler.
  DRAKE_UNREACHABLE();
}

template <typename T>
std::pair<T, int> SapSolver<T>::PerformExactLineSearch(
    const systems::Context<T>&, const SearchDirectionData&,
    systems::Context<T>*) const {
  throw std::logic_error(
      "SapSolver::PerformExactLineSearch(): Only T = double is supported.");
}


template <>
std::pair<double, int> SapSolver<double>::PerformExactLineSearch(
    const systems::Context<double>& context,
    const SearchDirectionData& search_direction_data,
    systems::Context<double>* scratch) const {
  using std::abs;

  // Ensure everythin is allocated beyond this point.
  model_->EvalConstraintsHessian(*scratch);
  model_->EvalImpulses(*scratch);

  // ===========================================================================
  // Copy/paste from PerformBackTrackingLineSearch(). Consider refactoring.
  // ===========================================================================

  // Line search parameters.
  //const double rho = parameters_.ls_rho;
  //const double c = parameters_.ls_c;
  //const int max_iterations = parameters_.ls_max_iterations;

  // Quantities at alpha = 0.
  const double ell0 = model_->EvalCost(context);
  const VectorX<double>& ell_grad_v0 = model_->EvalCostGradient(context);

  // dℓ/dα(α = 0) = ∇ᵥℓ(α = 0)⋅Δv.
  const VectorX<double>& dv = search_direction_data.dv;
  const double dell_dalpha0 = ell_grad_v0.dot(dv);

  // Cost and gradient at alpha_max.
  double dell, d2ell;
  VectorX<double> vec_scratch;
  const double ell_alpha_max =
      CalcCostAlongLine(context, search_direction_data,
                        parameters_.ls_alpha_max, scratch, &vec_scratch, &dell, &d2ell);
  // If the cost keeps decreasing at alpha_max, then alpha = alpha_max is a good
  // step size.                        
  if (dell <= 0) return std::make_pair(parameters_.ls_alpha_max, 0);

  // TODO(amcastro-tri): estimate tolerance that takes into account the problem
  // size and therefor the expected precision loss during matrix-vector
  // multiplications (when using block sparse matrices).
  const double kTolerance = 50 * std::numeric_limits<double>::epsilon();
  // N.B. We expect ell_scale != 0 since otherwise SAP's optimality condition
  // would've been reached and the solver would not reach this point.
  // N.B. ell = 0 implies v = v* and gamma = 0, for which the momentum residual
  // is zero.
  // Given that the Hessian in SAP is SPD we know that dell_dalpha0 < 0
  // (strictly). dell_dalpha0 = 0 would mean that we reached the optimum but
  // most certainly due to round-off errors or very tight user specified
  // optimality tolerances, the optimality condition was not met and SAP
  // performed an additional iteration to find a search direction that, most
  // likely, is close to zero. We therefore detect this case with dell_dalpha0 ≈
  // 0 and accept the search direction with alpha = 1.
  const double ell_scale = 0.5 * (ell_alpha_max + ell0);
  if (abs(dell_dalpha0 / ell_scale) < kTolerance) return std::make_pair(1.0, 0);
  //PRINT_VAR(ell_scale);
  //PRINT_VAR(abs(dell_dalpha0 / ell_scale));

  // dℓ/dα(α = 0) is guaranteed to be strictly negative given the the Hessian of
  // the cost is positive definite. Only round-off errors in the factorization
  // of the Hessian for ill-conditioned systems (small regularization) can
  // destroy this property. If so, we abort given that'd mean the model must be
  // revisited.
  if (dell_dalpha0 >= 0) {
    throw std::runtime_error(
        "The cost does not decrease along the search direction. This is "
        "usually caused by an excessive accumulation round-off errors for "
        "ill-conditioned systems. Consider revisiting your model.");
  }

  // ===========================================================================
  // End of copy/pasta.
  // ===========================================================================

  // N.B. We place the data needed to evaluate cost and gradients into a single
  // struct so that cost_and_gradient only needs to capture a single pointer.
  // This avoid heap allocation when calling RtSafe.
  struct EvalData {
    const SapSolver<double>* solver;
    const Context<double>& context0;  // Context at alpha = 0.
    const SearchDirectionData& search_direction_data;
    Context<double>& scratch;   // Context at alpha != 0.
    const double ell_scale;
    VectorX<double> vec_scratch;
  };
  EvalData data{this, context, search_direction_data, *scratch, ell_scale};

  auto cost_and_gradient = [&data](double x) {
    // We normalize as:
    // ell_tilde = (ell-ell_min)/ell_delta

    // x == alpha
    // f == dell_dalpha
    // dfdx == d2ell_dalpha2
    double dell_dalpha;
    double d2ell_dalpha2;
    data.solver->CalcCostAlongLine(data.context0, data.search_direction_data, x,
                                   &data.scratch, &data.vec_scratch, &dell_dalpha, &d2ell_dalpha2);

    //PRINT_VAR(x);
    //PRINT_VAR(dell_dalpha / data.ell_scale);
    //PRINT_VAR(d2ell_dalpha2 / data.ell_scale);

    return std::make_pair(dell_dalpha / data.ell_scale,
                          d2ell_dalpha2 / data.ell_scale);
  };

  //  LimitMalloc guard({.max_num_allocations = 0});

  // The most likely solution close to the optimal solution.
  const double alpha_guess = 1.0;

  // N.B. If we are here, then we already know that dell_dalpha0 < 0 and dell >
  // 0, and therefore [0, alpha_max] is a valid bracket. Otherwise we would've
  // already returned or thrown an exception due to numerical errors.
  const Bracket bracket(0., dell_dalpha0 / ell_scale, parameters_.ls_alpha_max,
                  dell / ell_scale);

  //std::cout << "Calling DoNewtonWithBisectionFallback():\n";
  const double x_rel_tol = parameters_.ls_rel_tolerance;
  const auto [alpha, num_iters] = DoNewtonWithBisectionFallback(
      cost_and_gradient, bracket, alpha_guess, x_rel_tol, kTolerance, 100);
  //std::cout << std::endl;      

  return std::make_pair(alpha, num_iters);
}

template <typename T>
MatrixX<T> SapSolver<T>::CalcDenseHessian(const Context<T>& context) const {
  // Explicitly build dense Hessian.
  // These matrices could be saved in the cache. However this method is only
  // intended as an alternative for debugging and optimizing it might not be
  // worth it.
  const int nv = model_->num_velocities();
  const int nk = model_->num_constraint_equations();

  // Make dense dynamics matrix.
  const std::vector<MatrixX<T>>& Acliques = model_->dynamics_matrix();
  MatrixX<T> Adense = MatrixX<T>::Zero(nv, nv);
  int offset = 0;
  for (const auto& Ac : Acliques) {
    const int nv_clique = Ac.rows();
    Adense.block(offset, offset, nv_clique, nv_clique) = Ac;
    offset += nv_clique;
  }

  // Make dense Jacobian matrix.
  const MatrixX<T> Jdense = model_->constraints_bundle().J().MakeDenseMatrix();

  // Make dense Hessian matrix G.
  const std::vector<MatrixX<T>>& G = model_->EvalConstraintsHessian(context);
  MatrixX<T> Gdense = MatrixX<T>::Zero(nk, nk);
  offset = 0;
  for (const auto& Gi : G) {
    const int ni = Gi.rows();
    Gdense.block(offset, offset, ni, ni) = Gi;
    offset += ni;
  }

  const MatrixX<T> H = Adense + Jdense.transpose() * Gdense * Jdense;

  return H;
}

template <typename T>
std::unique_ptr<SuperNodalSolver> SapSolver<T>::MakeSuperNodalSolver() const {
  if constexpr (std::is_same_v<T, double>) {
    const BlockSparseMatrix<T>& J = model_->constraints_bundle().J();
    return std::make_unique<SuperNodalSolver>(J.block_rows(), J.get_blocks(),
                                              model_->dynamics_matrix());
  } else {
    throw std::logic_error(
        "SapSolver::MakeSuperNodalSolver(): SuperNodalSolver only supports T "
        "= double.");
  }
}

template <typename T>
void SapSolver<T>::CallDenseSolver(const Context<T>& context,
                                   VectorX<T>* dv) const {
  const MatrixX<T> H = CalcDenseHessian(context);

  // Factorize Hessian.
  // TODO(amcastro-tri): when T = AutoDiffXd propagate gradients analytically
  // using the chain rule so that here we can use T = double for performance.
  // N.B. The support for dense algebra is mostly for testing purposes, even
  // though the computation of the dense H (and in particular of the Jᵀ⋅G⋅J
  // term) is very costly. Therefore below we decided to trade off speed for
  // stability when choosing to use an LDLT decomposition instead of a slightly
  // faster, though less stable, LLT decomposition.
  const math::LinearSolver<Eigen::LDLT, MatrixX<T>> H_ldlt(H);
  if (H_ldlt.eigen_linear_solver().info() != Eigen::Success) {
    // TODO(amcastro-tri): Unit test this condition.
    throw std::runtime_error("Dense LDLT factorization of the Hessian failed.");
  }

  // Compute search direction.
  const VectorX<T> rhs = -model_->EvalCostGradient(context);
  *dv = H_ldlt.Solve(rhs);
}

template <typename T>
void SapSolver<T>::UpdateSuperNodalSolver(
    const Context<T>& context, SuperNodalSolver* supernodal_solver) const {
  if constexpr (std::is_same_v<T, double>) {
    const std::vector<MatrixX<double>>& G =
        model_->EvalConstraintsHessian(context);
    supernodal_solver->SetWeightMatrix(G);
  } else {
    unused(context);
    unused(supernodal_solver);
    throw std::logic_error(
        "SapSolver::UpdateSuperNodalSolver(): SuperNodalSolver only supports T "
        "= double.");
  }
}

template <typename T>
void SapSolver<T>::CallSuperNodalSolver(const Context<T>& context,
                                        SuperNodalSolver* supernodal_solver,
                                        VectorX<T>* dv) const {
  if constexpr (std::is_same_v<T, double>) {
    UpdateSuperNodalSolver(context, supernodal_solver);
    if (!supernodal_solver->Factor()) {
      throw std::logic_error("SapSolver: Supernodal factorization failed.");
    }
    // We solve in place to avoid heap allocating additional memory for the
    // right hand side.
    *dv = -model_->EvalCostGradient(context);
    supernodal_solver->SolveInPlace(dv);
  } else {
    unused(context);
    unused(supernodal_solver);
    unused(dv);
    throw std::logic_error(
        "SapSolver::CallSuperNodalSolver(): SuperNodalSolver only supports T "
        "= double.");
  }
}

template <typename T>
void SapSolver<T>::CalcSearchDirectionData(
    const systems::Context<T>& context,
    SuperNodalSolver* supernodal_solver,
    SapSolver<T>::SearchDirectionData* data) const {
  DRAKE_DEMAND(parameters_.use_dense_algebra || (supernodal_solver != nullptr));
  // Update search direction dv.
  StopWatch watch;
  if (!parameters_.use_dense_algebra) {
    CallSuperNodalSolver(context, supernodal_solver, &data->dv);
  } else {
    CallDenseSolver(context, &data->dv);
  }
  stats_.solve_time += watch.Elapsed();

  // Update Δp, Δvc and d²ellA/dα².
  model_->constraints_bundle().J().Multiply(data->dv, &data->dvc);
  model_->MultiplyByDynamicsMatrix(data->dv, &data->dp);
  data->d2ellA_dalpha2 = data->dv.dot(data->dp);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapSolver)
