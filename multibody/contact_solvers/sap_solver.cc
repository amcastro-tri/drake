#include "drake/multibody/contact_solvers/sap_solver.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <tuple>
#include <utility>

#include "fmt/format.h"

#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/contact_solvers/cone_ray_intersect.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/geodesic_interior_point_method.h"
#include "drake/multibody/contact_solvers/rtsafe.h"
#include "drake/multibody/contact_solvers/supernodal_solver.h"

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using Eigen::SparseMatrix;
using Eigen::SparseVector;

template <typename T>
SapSolver<T>::SapSolver()
    : ConvexSolverBase<T>({SapSolverParameters().theta,
                           SapSolverParameters().Rt_factor,
                           SapSolverParameters().alpha,
                           SapSolverParameters().sigma}) {}

template <typename T>
ContactSolverStatus SapSolver<T>::DoSolveWithGuess(
    const typename ConvexSolverBase<T>::PreProcessedData& data,
    const VectorX<T>& v_guess, ContactSolverResults<T>* result) {
  throw std::logic_error("Only T = double is supported.");
}

template <>
ContactSolverStatus SapSolver<double>::DoSolveWithGuess(
    const ConvexSolverBase<double>::PreProcessedData& data,
    const VectorX<double>& v_guess, ContactSolverResults<double>* results) {
  using std::abs;
  using std::max;

  const auto& dynamics_data = *data.dynamics_data;
  const auto& contact_data = *data.contact_data;

  const int nv = dynamics_data.num_velocities();
  const int nc = contact_data.num_contacts();
  const int nc3 = 3 * nc;

  // The primal method needs the inverse dynamics data.
  DRAKE_DEMAND(dynamics_data.has_inverse_dynamics());

  // We should not attempt solving zero sized problems for no reason since the
  // solution is trivially v = v*.
  DRAKE_DEMAND(nc != 0);

  // Print stuff for debugging.
  // TODO: refactor into PrintProblemSize().
  if (parameters_.verbosity_level >= 1) {
    PRINT_VAR(nv);
    PRINT_VAR(nc);
    PRINT_VAR(data_.Mt.size());
    PRINT_VAR(data_.Mblock.rows());
    PRINT_VAR(data_.Mblock.cols());
    PRINT_VAR(data_.Mblock.num_blocks());

    PRINT_VAR(data_.Jblock.block_rows());
    PRINT_VAR(data_.Jblock.block_cols());
    PRINT_VAR(data_.Jblock.rows());
    PRINT_VAR(data_.Jblock.cols());
    PRINT_VAR(data_.Jblock.num_blocks());
  }
  // TODO: refactor into PrintProblemStructure().
  if (parameters_.verbosity_level >= 2) {
    for (const auto& [p, t, Jb] : data_.Jblock.get_blocks()) {
      std::cout << fmt::format("(p,t) = ({:d},{:d}). {:d}x{:d}.\n", p, t,
                               Jb.rows(), Jb.cols());
    }
  }

  State state(nv, nc, parameters_.compare_with_dense);
  aux_state_.Resize(nv, nc, parameters_.compare_with_dense);
  workspace_.Resize(nv, nc);
  VectorX<double> gamma_id(3 * nc);
  VectorX<double> v_work1(nv);
  VectorX<double> v_work2(nv);
  VectorX<double> v_work3(nv);
  VectorX<double> xc_work1(3 * nc);

  state.mutable_v() = v_guess;
  // Compute velocity and impulses here to use in the computation of convergence
  // metrics later for the very first iteration.
  auto& cache = state.mutable_cache();
  CalcVelocityAndImpulses(state, &cache.vc, &cache.gamma);

  // Previous iteration state, for error computation and reporting.
  State state_kp = state;

  const bool use_geodesic_solver = parameters_.use_geodesic_solver;
  if (use_geodesic_solver) {
    GeodesicSolverSolution sol;
    GeodesicSolverOptions options;
    options.target_mu = 1e-5;
    options.maximum_iterations = 500;
    options.verbosity = parameters_.verbosity_level;

    sol.v = state.mutable_v();
    sol.lambda = cache.gamma;

    const auto& vc_stab = data_.vc_stab;
    const auto& v_star = data_.dynamics_data->get_v_star();

    // solution_geo.v contains optimal velocity
    // solution_geo.info.iterations contains executed iterations.
    // TODO(amcastro-tri): expose solver parameters controlling the accuracy of
    // the solution.
    auto solution_geo =
        GeodesicSolver(sol, contact_data.get_mu(),
                       data_.Jblock.get_blocks(), data_.Mt, data_.R,
                       data_.Jblock.block_rows(), data_.nc, v_star, vc_stab, options);

    if (solution_geo.info.failed) return ContactSolverStatus::kFailure;

    state.mutable_v() = solution_geo.v;
    const auto& Jop = contact_data.get_Jc();
    Jop.Multiply(state.v(), &cache.vc);

    cache.gamma = solution_geo.lambda;
    PackContactResults(data_, state.v(), cache.vc, cache.gamma, results);

    // Most of the metrics are garbage for geodesic solver.
    SapSolverIterationMetrics metrics;
    std::tie(metrics.mom_rel_l2, metrics.mom_rel_max) =
        this->CalcRelativeMomentumError(data, state.v(), cache.gamma);
    this->CalcEnergyMetrics(data, state.v(), cache.gamma, &metrics.Ek,
                            &metrics.costM, &metrics.costR, &metrics.cost);
    this->CalcSlipMetrics(data, cache.vc, &metrics.vt_mean, &metrics.vt_rms);
    metrics.opt_cond =
        this->CalcOptimalityCondition(data_, state.v(), cache.gamma, &xc_work1);
    this->CalcAnalyticalInverseDynamics(parameters_.soft_tolerance, cache.vc,
                                        &gamma_id);
    metrics.id_rel_error = (cache.gamma - gamma_id).norm() /
                           max(cache.gamma.norm(), gamma_id.norm());

    return ContactSolverStatus::kSuccess;

  } else {
    // Super nodal solver is constructed once per time-step to reuse structure
    // of M and J.
    std::unique_ptr<conex::SuperNodalSolver> solver;    

    double alpha = 1.0;
    int num_ls_iters = 0;  // Count line-search iterations.

    // Start Newton iterations.
    int k = 0;
    int num_iterations = 0;
    for (; k < parameters_.max_iterations; ++k) {
      if (parameters_.verbosity_level >= 3) {
        std::cout << std::string(80, '=') << std::endl;
        std::cout << std::string(80, '=') << std::endl;
        std::cout << "Iteration: " << k << std::endl;
      }

 
    // N.B. This is update vc and gamma in the cache!
    // Therefore the call must not be removed!
    // TODO: replace by proper call to only do what we need.
    CalcIterationMetrics(state, state_kp, num_ls_iters, alpha, &xc_work1);
      
      double scaled_momentum_error, momentum_scale;
      {
        double Ek, costM, costR, cost;
        CalcScaledMomentumAndScales(
            data, state.v(), cache.gamma, &scaled_momentum_error,
            &momentum_scale, &Ek, &costM, &costR,
            &cost, &v_work1, &v_work2, &v_work3);
      }
      // Note: only update the useful stats. Remove things like mom_rel_max.
      if (scaled_momentum_error <= parameters_.rel_tolerance * momentum_scale) {
        // TODO: refactor into PrintConvergedIterationStats().
        if (parameters_.verbosity_level >= 1) {
          std::cout << "Iteration converged at: " << k << std::endl;
          std::cout << "ell: " << cache.ell << std::endl;
          PRINT_VAR(cache.vc.norm());
          std::cout << std::string(80, '=') << std::endl;
        }
        break;
      } else {
        ++num_iterations;  // For statistics, we only count those iterations
                           // that actually do work, i.e. solve a system of
                           // linear equations.
        // Prepare supernodal solver on first iteration only when needed.
        // That is, if converged, avoid this work.
        if (parameters_.use_supernodal_solver && k == 0) {
          solver = std::make_unique<conex::SuperNodalSolver>(
              data_.Jblock.block_rows(), data_.Jblock.get_blocks(), data_.Mt);
        }
      }
      state_kp = state;

      // Assembly happens withing these calls to CallSupernodalSolver() and
      // CallDenseSolver() so that we can factor the assembly effort of the
      // supernodal solver in the same timer.
      // TODO: get rid of CallDenseSolver(). Only here for debbuging.
      if (parameters_.use_supernodal_solver) {
        CallSupernodalSolver(state, &cache.dv, solver.get());
      } else {
        CallDenseSolver(state, &cache.dv);
      }
      // The cost must always go down.
      if (k > 0) {
        DRAKE_DEMAND(cache.ell < state_kp.cache().ell);
      }

      // Update change in contact velocities.
      const auto& Jop = contact_data.get_Jc();
      Jop.Multiply(cache.dv, &cache.dvc);
      cache.valid_search_direction = true;  // both dv and dvc are now valid.

      // TODO: add convergence check, even if only for statistics on the scaled
      // moementum balance, i.e. r = D * (M(v-v*)-Jᵀγ), with D = 1/
      // sqrt(diag(M)).
      // TODO: consider updating vc and gamma here for convergece criterias.
      // Cheap if no dgamma_dy is computed.      

      // Perform line-search.
      // N.B. If converged, we allow one last update with alpha = 1.0.
      alpha = 1.0;
      num_ls_iters = 0;  // Count line-search iterations.
      // remove this when you fully swap to the optimality condition for
      // convergence criteria.
      bool converged = false;
      if (!converged) {
        // If not converged, we know dvc !=0 and therefore we have a valid
        // search direction for line search.        
        num_ls_iters = CalcInexactLineSearchParameter(state, &alpha);        
      }

      // Update state.
      state.mutable_v() += alpha * cache.dv;

      // TODO: refactor into PrintNewtonStats().
      if (parameters_.verbosity_level >= 3) {
        PRINT_VAR(cache.ellM);
        PRINT_VAR(cache.ell);
        PRINT_VAR(cache.dv.norm());
        PRINT_VAR(state_kp.cache().ell);
        PRINT_VAR(converged);
        PRINT_VAR(alpha);
      }
    }

    if (k == parameters_.max_iterations) return ContactSolverStatus::kFailure;

    PackContactResults(data_, state.v(), cache.vc, cache.gamma, results);

    return ContactSolverStatus::kSuccess;
  }
}

template <typename T>
void SapSolver<T>::CalcVelocityAndImpulses(
    const State& state, VectorX<T>* vc, VectorX<T>* gamma,
    std::vector<Matrix3<T>>* dgamma_dy) const {
  // If dgamma_dy != nullptr, is because we are trying to compute gradients.
  // We'll therefore proceed with the computation.
  if (state.cache().valid_contact_velocity_and_impulses && dgamma_dy == nullptr)
    return;

  // Update contact velocity.
  const auto& Jc = data_.contact_data->get_Jc();
  Jc.Multiply(state.v(), &*vc);

  // Update impulse (and gradients if G != nullptr).
  this->CalcAnalyticalInverseDynamics(parameters_.soft_tolerance, *vc, gamma,
                                      dgamma_dy,
                                      &state.mutable_cache().regions);
  state.mutable_cache().valid_contact_velocity_and_impulses = true;
}

template <typename T>
T SapSolver<T>::CalcCostAndGradients(
    const State& state, VectorX<T>* ell_grad_v, std::vector<MatrixX<T>>* G,
    T* ellM_out, T* ellR_out, MatrixX<T>* ell_hessian_v) const {
  DRAKE_DEMAND(ell_grad_v != nullptr);
  DRAKE_DEMAND(G != nullptr);
  if (state.cache().valid_cost_and_gradients) return state.cache().ell;

  // Evaluate velocity cache.
  auto& cache = state.mutable_cache();
  CalcVelocityAndImpulses(state, &cache.vc, &cache.gamma, &cache.dgamma_dy);

  // Aliases to data.
  const int nv = data_.nv;
  const int nc = data_.nc;
  const auto& Aop = data_.dynamics_data->get_A();
  const auto& Jop = data_.contact_data->get_Jc();
  const auto& R = data_.R;
  const auto& vc_stab = data_.vc_stab;
  const auto& v_star = data_.dynamics_data->get_v_star();

  // Workspace.
  VectorX<T>& Mv = workspace_.aux_v1;
  VectorX<T>& dv = workspace_.aux_v2;

  // Cost:
  // ℓ(v) = 0.5⋅(v−v*)ᵀM(v−v*) + 0.5⋅γ(v)ᵀ R γ(v).
  const auto& gamma = cache.gamma;
  const T ellR = 0.5 * gamma.dot(R.asDiagonal() * gamma);
  dv = state.v() - v_star;
  Aop.Multiply(dv, &Mv);
  const T ellM = 0.5 * dv.dot(Mv);
  const T ell = ellM + ellR;
  if (ellM_out) *ellM_out = ellM;
  if (ellR_out) *ellR_out = ellR;

  // Gradient:
  // ∇ℓ(v) = M(v−v*) − Jᵀγ.
  if (ell_grad_v) {
    ell_grad_v->setZero();
    Jop.MultiplyByTranspose(gamma, ell_grad_v);  // ell_grad_v= J^T * gamma.
    (*ell_grad_v) = -(*ell_grad_v);
    (*ell_grad_v) += Mv;
  }

  const auto& dgamma_dy = cache.dgamma_dy;
  for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    const auto& R_ic = R.template segment<3>(ic3);
    const Vector3<T> Rinv = R_ic.cwiseInverse();
    const Matrix3<T>& dgamma_dy_ic = dgamma_dy[ic];
    MatrixX<T>& G_ic = (*G)[ic];
    G_ic = dgamma_dy_ic * Rinv.asDiagonal();
  }

  // We don't build the Hessian here anymore.
  // This is only for debugging.
  if (ell_hessian_v &&
      (!parameters_.use_supernodal_solver ||
       (parameters_.use_supernodal_solver && parameters_.compare_with_dense))) {
    MatrixX<T> Jdense(3 * nc, nv);
    Jop.AssembleMatrix(&Jdense);
    MatrixX<T> Adense(nv, nv);
    Aop.AssembleMatrix(&Adense);

    MatrixX<T> GJ(3 * nc, nv);
    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      const MatrixX<T>& G_ic = (*G)[ic];
      GJ.block(ic3, 0, 3, nv) = G_ic * Jdense.block(ic3, 0, 3, nv);
    }
    *ell_hessian_v = Adense + Jdense.transpose() * GJ;

    cache.valid_dense_gradients = true;
  }

  cache.valid_cost_and_gradients = true;

  return ell;
}

template <typename T>
T SapSolver<T>::CalcLineSearchCostAndDerivatives(
    const State& state_v, const T& alpha, T* dell_dalpha, T* d2ell_dalpha2,
    State* state_alpha, T* ellM_out, T* dellM_dalpha_out, T* d2ellM_dalpha2_out,
    T* ellR_out, T* dellR_dalpha_out, T* d2ellR_dalpha2_out) const {
  DRAKE_DEMAND(state_v.cache().valid_line_search_quantities);

  // Data.
  const int nc = data_.nc;
  const auto& R = data_.R;
  const auto& Rinv = data_.Rinv;
  const auto& v_star = data_.dynamics_data->get_v_star();

  // Quantities at state v.
  const auto& dv = state_v.cache().dv;
  const auto& dp = state_v.cache().dp;
  const auto& dvc = state_v.cache().dvc;

  // State at v(alpha).
  state_alpha->mutable_v() = state_v.v() + alpha * dv;
  const auto& v = state_alpha->v();
  const auto& gamma = state_alpha->cache().gamma;  // We'll update below.
  const auto& dgamma_dy = state_alpha->cache().dgamma_dy;

  // Update velocities and impulses at v(alpha).
  CalcVelocityAndImpulses(*state_alpha, &state_alpha->mutable_cache().vc,
                          &state_alpha->mutable_cache().gamma,
                          &state_alpha->mutable_cache().dgamma_dy);

  // Cost ellR.
  const T ellR = 0.5 * gamma.dot(R.asDiagonal() * gamma);

  // We can compute ellM in terms of precomputed terms.
  T ellM = state_v.cache().ellM;
  ellM += alpha * dp.dot(state_v.v() - v_star);
  ellM += 0.5 * alpha * alpha * state_v.cache().d2ellM_dalpha2;
  const T ell = ellM + ellR;

  // If dell_dalpha == nullptr, it is because we are only requesting the cost.
  // We are done and return.
  if (dell_dalpha == nullptr) return ell;

  if (ellM_out) *ellM_out = ellM;
  if (ellM_out) *ellR_out = ellR;

  // First derivative.
  const T dellM_dalpha = dp.dot(v - v_star);
  const T dellR_dalpha = -dvc.dot(gamma);
  *dell_dalpha = dellM_dalpha + dellR_dalpha;

  if (dellM_dalpha_out) *dellM_dalpha_out = dellM_dalpha;
  if (dellR_dalpha_out) *dellR_dalpha_out = dellR_dalpha;

  // Second derivative.
  const T d2ellM_dalpha2 = state_v.cache().d2ellM_dalpha2;
  DRAKE_DEMAND(d2ellM_dalpha2 > 0.0);

  T d2ellR_dalpha2 = 0;  // = −∇vcᵀ⋅dγ/dvc⋅∇vc
  for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    // const Vector3<T> Rinv = R.cwiseInverse();
    const auto Rinv_ic = Rinv.template segment<3>(ic3);
    const Matrix3<T>& dgamma_dy_ic = dgamma_dy[ic];
    const auto dvc_ic = dvc.template segment<3>(ic3);

    const Matrix3<T> dgamma_dvc = -dgamma_dy_ic * Rinv_ic.asDiagonal();

    const T d2ellR_dalpha2_ic = -dvc_ic.transpose() * dgamma_dvc * dvc_ic;
    // Allow certain slop for the condition d2ellR_dalpha2 >= 0.
    if (d2ellR_dalpha2_ic < -1.0e-15) {
      Eigen::IOFormat OctaveFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "",
                                "[", "]");
      PRINT_VAR(d2ellR_dalpha2_ic);
      PRINT_VARn(dgamma_dvc);
      std::cout << "OctaveFmt\n";
      std::cout << dgamma_dvc.format(OctaveFmt) << std::endl;
      PRINT_VAR(dvc_ic.transpose());
      throw std::runtime_error("d2ellR_dalpha2_ic<0");
    }

    // clip any machine epsilon number smaller than zero to avoid accumulation
    // of small negative numbers. This is ok since we checked above that
    // d2ellR_dalpha2_ic is positive within a slop tolerance.
    using std::max;
    d2ellR_dalpha2 += max(0.0, d2ellR_dalpha2_ic);
  }
  DRAKE_DEMAND(d2ellR_dalpha2 >= 0.0);

  *d2ell_dalpha2 = d2ellM_dalpha2 + d2ellR_dalpha2;
  DRAKE_DEMAND(*d2ell_dalpha2);

  if (d2ellM_dalpha2_out) *d2ellM_dalpha2_out = d2ellM_dalpha2;
  if (d2ellR_dalpha2_out) *d2ellR_dalpha2_out = d2ellR_dalpha2;

  return ell;
}

template <typename T>
int SapSolver<T>::CalcInexactLineSearchParameter(
    const State& state, T* alpha_out) const {
  DRAKE_DEMAND(state.cache().valid_cost_and_gradients);

  // Quantities at alpha = 0.
  const T ell0 = state.cache().ell;
  const auto& ell_grad_v0 = state.cache().ell_grad_v;

  // Search direction.
  const VectorX<T>& dv = state.cache().dv;

  // Parameters.
  const double rho = parameters_.ls_rho;
  const double c = parameters_.ls_c;
  const int max_iterations = parameters_.ls_max_iterations;

  // Update quantities that depend on dv used for
  // line-search.
  auto& cache = state.mutable_cache();
  const auto& Aop = data_.dynamics_data->get_A();
  Aop.Multiply(cache.dv, &cache.dp);  // M * cache.dv;
  cache.d2ellM_dalpha2 = cache.dv.dot(cache.dp);
  cache.valid_line_search_quantities = true;

  // State at v_alpha = v + alpha * dv.
  State state_alpha(state);

  // Save dot product between dv and ell_grad_v.
  const T dell_dalpha0 = ell_grad_v0.dot(dv);

  // If dell_dalpha at v is not negative, something went terribly wrong.
  DRAKE_DEMAND(dell_dalpha0 < 0);

  T alpha = parameters_.ls_alpha_max;
  T ell_alpha = CalcLineSearchCostAndDerivatives(state, alpha, nullptr, nullptr,
                                                 &state_alpha);
  T alpha_prev = alpha;
  T ell_prev = ell_alpha;

  // Record if the previous iteration satisfies the Armijo condition.
  bool satisfies_armijo_prev = ell_alpha < ell0 + c * alpha * dell_dalpha0;

  int num_iters = 0;
  for (int iter = 0; iter < max_iterations; ++iter) {
    ++num_iters;
    alpha *= rho;
    ell_alpha = CalcLineSearchCostAndDerivatives(state, alpha, nullptr, nullptr,
                                                 &state_alpha);
    const bool satisfies_armijo = ell_alpha < ell0 + c * alpha * dell_dalpha0;
    // std::cout << alpha << " " << ell_alpha - ell0 << " " << satisfies_armijo
    //          << std::endl;
    // Armijo's criteria.
    // Since we know the function is convex, we in addition continue iterating
    // until ell starts increasing.
    if (ell_alpha > ell_prev) {
      if (satisfies_armijo) {
        // We don't go back one because we do know that the current alpha
        // satisfies the Armijo condition.
        // If the previous iterate satisfies the Armijo condition, it is better
        // since ell_prev < ell_alpha.
        if (satisfies_armijo_prev) alpha /= rho;
        // value.
        *alpha_out = alpha;
        return num_iters;
      } else {
        throw std::runtime_error("Line search failed.");
      }
    }
    alpha_prev = alpha;
    ell_prev = ell_alpha;
    satisfies_armijo_prev = satisfies_armijo;
  }
  throw std::runtime_error("Line search reached max iterations.");
  DRAKE_UNREACHABLE();
}

template <typename T>
std::vector<T> SapSolver<T>::FindAllContinuousIntervals(
    const State& state, const VectorX<T>& y, const VectorX<T>& dy) const {
  // Pre-processed data.
  const int nc = data_.nc;
  const auto& R = data_.R;

  // Problem data.
  const auto& mu_all = data_.contact_data->get_mu();

  const double soft_tolerance = parameters_.soft_tolerance;
  const double alpha_max = parameters_.ls_alpha_max;

  std::vector<T> cone_crossings;

  // We want the first interval to start at 0. We add it.
  cone_crossings.push_back(0.0);

  // We reserve the maximum number of possible crossings to avoid heap
  // allocation in the loop.
  // There can be up to two intersections per cone. Therefore if a ray
  // intersects both the cone and its dual twice, we have a maximum of four
  // intersections per contact.
  cone_crossings.reserve(4 * nc);
  int num_alphas;
  Vector2<T> alphas;
  for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    const auto& y_ic = y.template segment<3>(ic3);
    const auto& dy_ic = dy.template segment<3>(ic3);
    const auto& R_ic = R.template segment<3>(ic3);
    const T& mu = mu_all(ic);
    const T& Rt = R_ic(0);
    const T& Rn = R_ic(2);
    const T mu_hat = mu * Rt / Rn;

    // Intersection with the friction cone s(y) = mu*yn - yr >= 0.
    num_alphas =
        CalcRayConeIntersection(mu, soft_tolerance, y_ic, dy_ic, &alphas);
    if (num_alphas >= 1 && alphas[0] < alpha_max)
      cone_crossings.push_back(alphas[0]);
    if (num_alphas == 2 && alphas[1] < alpha_max)
      cone_crossings.push_back(alphas[1]);

    // Intersection with the polar cone. yn < -mu_hat*yr. Since the polar cone
    // is in the negative z half space and CalcRayConeIntersection() only works
    // with cones on the positive z half space, we mirror y to y_minus and find
    // the intersection with the (now positive) polar cone
    // s(y) = yn/mu_hat - yr >= 0.
    const Vector3<T> y_minus(y_ic(0), y_ic(1), -y_ic(2));
    const Vector3<T> dy_minus(dy_ic(0), dy_ic(1), -dy_ic(2));
    num_alphas = CalcRayConeIntersection(1.0 / mu_hat, soft_tolerance, y_minus,
                                         dy_minus, &alphas);
    if (num_alphas >= 1 && alphas[0] < alpha_max)
      cone_crossings.push_back(alphas[0]);
    if (num_alphas == 2 && alphas[1] < alpha_max)
      cone_crossings.push_back(alphas[1]);
  }

  // We want the last interval to end at alpha_max. We add it.
  cone_crossings.push_back(parameters_.ls_alpha_max);

  // std::unique eliminates all except the first element from every consecutive
  // group of equivalent elements. Therefore to remove all repeated elements, we
  // must first sort the vector.
  std::sort(cone_crossings.begin(), cone_crossings.end());

  // Remove repeated entries (within a given tolerance.)
  auto pred = [](const T& a, const T& b) -> bool {
    constexpr double kEqualityTolerance = 1.0e-10;
    using std::abs;
    return (abs(a - b) < kEqualityTolerance);
  };
  auto last = std::unique(cone_crossings.begin(), cone_crossings.end(), pred);

  // Erases unspecified values to reduces the physical size of cone_crossings.
  cone_crossings.erase(last, cone_crossings.end());

  for (double alpha_cross : cone_crossings) {
    PRINT_VAR(alpha_cross);
  }

  return cone_crossings;
}

template <typename T>
SapSolverIterationMetrics
SapSolver<T>::CalcIterationMetrics(const State& s,
                                                   const State& s0,
                                                   int num_ls_iterations,
                                                   double alpha,
                                                   VectorX<T>* xc_work1) const {
  using std::max;

  // Update velocity and impulses for reporting.
  auto& cache = s.mutable_cache();
  CalcVelocityAndImpulses(s, &cache.vc, &cache.gamma);

  SapSolverIterationMetrics metrics;
  metrics.vc_error_max_norm =
      (s.cache().vc - s0.cache().vc).template lpNorm<Eigen::Infinity>();
  metrics.v_error_max_norm =
      (s.v() - s0.v()).template lpNorm<Eigen::Infinity>();
  metrics.gamma_error_max_norm =
      (s.cache().gamma - s0.cache().gamma).template lpNorm<Eigen::Infinity>();
  // For this primal solver, the inverse dynamics relative error reduces to the
  // computation of the relative error between successive iterations.
  metrics.id_rel_error = (s.cache().gamma - s0.cache().gamma).norm() /
                         max(s.cache().gamma.norm(), s0.cache().gamma.norm());
  metrics.ellR = s.cache().ellR;
  metrics.ell = s.cache().ell;
  metrics.grad_ell_max_norm =
      s.cache().ell_grad_v.template lpNorm<Eigen::Infinity>();
  metrics.search_direction_max_norm =
      s.cache().dv.template lpNorm<Eigen::Infinity>();
  metrics.ls_iters = num_ls_iterations;
  metrics.ls_alpha = alpha;
  metrics.rcond = s.cache().condition_number;
  metrics.opt_cond =
      this->CalcOptimalityCondition(data_, s.v(), cache.gamma, xc_work1);
  metrics.gamma_norm = cache.gamma.norm();
  metrics.vc_norm = cache.vc.norm();

  return metrics;
}

template <typename T>
void SapSolver<T>::CallSupernodalSolver(
    const State& s, VectorX<T>* dv, conex::SuperNodalSolver* solver) {
  auto& cache = s.mutable_cache();

  cache.ell = CalcCostAndGradients(s, &cache.ell_grad_v, &cache.G, &cache.ellM,
                                   &cache.ellR, &cache.ell_hessian_v);

  // This call does the actual assembly H = A + J G Jᵀ.
  solver->SetWeightMatrix(cache.G);

  // Build full matrix for debugging.
  if (parameters_.compare_with_dense) {
    const MatrixXd H = solver->FullMatrix();
    PRINT_VAR((cache.ell_hessian_v - H).norm());
    if ((cache.ell_hessian_v - H).norm() > 1.0e-2) {
      throw std::runtime_error(
          "Supernodal Hessian differs from dense algebra Hessian.");
    }
  }

  // Factor() overwrites the assembled matrix with its LLT decomposition.
  // We'll count it as part of the linear solver time.
  solver->Factor();
  *dv = -cache.ell_grad_v;  // we solve dv in place.
  solver->SolveInPlace(dv);
}

template <typename T>
void SapSolver<T>::CallDenseSolver(const State& state,
                                                   VectorX<T>* dv) {
  const int nv = data_.nv;

  auto& cache = state.mutable_cache();
  cache.ell =
      CalcCostAndGradients(state, &cache.ell_grad_v, &cache.G, &cache.ellM,
                           &cache.ellR, &cache.ell_hessian_v);

  // We'll use Jacobi preconditioning to improve H's condition number. This
  // greatly reduces round-off errors even when using direct solvers.
  // const VectorXd D = M.diagonal().cwiseSqrt().cwiseInverse();
  // const VectorXd D =
  //    cache.ell_hessian_v.diagonal().cwiseSqrt().cwiseInverse();
  const VectorXd D = VectorXd::Ones(nv);
  const VectorXd rhs = -(D.asDiagonal() * cache.ell_grad_v);
  const MatrixXd lhs = D.asDiagonal() * cache.ell_hessian_v * D.asDiagonal();

  // Factorize Hessian.
  Eigen::LDLT<MatrixXd> Hldlt(lhs);
  if (Hldlt.info() != Eigen::Success) {
    // return ContactSolverStatus::kFailure;
    throw std::runtime_error("LDLT solver failed.");
  }
  // We track the condition number for our records.
  cache.condition_number = Hldlt.rcond();
  PRINT_VAR(cache.condition_number);

  // Compute search direction.
  *dv = D.asDiagonal() * Hldlt.solve(rhs);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::
    SapSolver<double>;
