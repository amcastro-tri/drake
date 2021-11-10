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
#include "drake/common/unused.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
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
SapSolver<T>::SapSolver() {}

template <typename T>
ContactSolverStatus SapSolver<T>::SolveWithGuess(
    const T& time_step, const SystemDynamicsData<T>& dynamics_data,
    const PointContactData<T>& contact_data, const VectorX<T>& v_guess,
    ContactSolverResults<T>* results) {
  // The primal method needs the inverse dynamics data.
  DRAKE_DEMAND(dynamics_data.has_inverse_dynamics());
  // User code should only call the solver for problems with constraints.
  // Otherwise the solution is trivially v = v*.
  DRAKE_DEMAND(contact_data.num_contacts() != 0);
  data_ = PreProcessData(time_step, dynamics_data, contact_data);
  return DoSolveWithGuess(data_, v_guess, results);
}

template <typename T>
Vector3<T> SapSolver<T>::CalcProjection(
    const ProjectionParams& params, const Eigen::Ref<const Vector3<T>>& y,
    const T& yr, const T& yn, const Eigen::Ref<const Vector2<T>>& that,
    int* region, Matrix3<T>* dPdy) const {
  const T& mu = params.mu;
  const T& Rt = params.Rt;
  const T& Rn = params.Rn;
  const T mu_hat = mu * Rt / Rn;

  Vector3<T> gamma;
  // Analytical projection of y onto the friction cone ℱ using the R norm.
  if (yr < mu * yn) {  // Region I, stiction.
    *region = 1;
    gamma = y;
    if (dPdy) dPdy->setIdentity();
  } else if (-mu_hat * yr < yn && yn <= yr / mu) {  // Region II, sliding.
    *region = 2;
    // Common terms:
    const T mu_tilde2 = mu * mu_hat;  // mu_tilde = mu * sqrt(Rt/Rn).
    const T factor = 1.0 / (1.0 + mu_tilde2);

    // Projection P(y).
    const T gn = (yn + mu_hat * yr) * factor;
    const Vector2<T> gt = mu * gn * that;
    gamma.template head<2>() = gt;
    gamma(2) = gn;

    // Gradient:
    if (dPdy) {
      const Matrix2<T> P = that * that.transpose();
      const Matrix2<T> Pperp = Matrix2<T>::Identity() - P;

      // We split dPdy into separate blocks:
      //
      // dPdy = |dgt_dyt dgt_dyn|
      //        |dgn_dyt dgn_dyn|
      // where dgt_dyt ∈ ℝ²ˣ², dgt_dyn ∈ ℝ², dgn_dyt ∈ ℝ²ˣ¹ and dgn_dyn ∈ ℝ.
      const Matrix2<T> dgt_dyt = mu * (gn / yr * Pperp + mu_hat * factor * P);
      const Vector2<T> dgt_dyn = mu * factor * that;
      const RowVector2<T> dgn_dyt = mu_hat * factor * that.transpose();
      const T dgn_dyn = factor;

      dPdy->template topLeftCorner<2, 2>() = dgt_dyt;
      dPdy->template topRightCorner<2, 1>() = dgt_dyn;
      dPdy->template bottomLeftCorner<1, 2>() = dgn_dyt;
      (*dPdy)(2, 2) = dgn_dyn;
    }
  } else {  // yn <= -mu_hat * yr
    *region = 3;
    // Region III, no contact.
    gamma.setZero();
    if (dPdy) dPdy->setZero();
  }

  return gamma;
}

template <typename T>
void SapSolver<T>::CalcDelassusDiagonalApproximation(
    int nc, const std::vector<MatrixX<T>>& Mt,
    const BlockSparseMatrix<T>& Jblock, VectorX<T>* Wdiag) const {
  DRAKE_DEMAND(Wdiag != nullptr);
  DRAKE_DEMAND(Wdiag->size() == nc);
  const int nt = Mt.size();  // Number of trees.

  // We compute a factorization of M once so we can re-use it multiple times
  // below.
  std::vector<Eigen::LLT<MatrixX<T>>> M_ldlt;
  M_ldlt.resize(nt);
  for (int t = 0; t < nt; ++t) {
    const auto& Mt_local = Mt[t];
    M_ldlt[t] = Mt_local.llt();
  }

  // We compute a diagonal approximation to the Delassus operator W. We
  // initialize it to zero and progressively add contributions in an O(n) pass.
  std::vector<Matrix3<T>> W(nc, Matrix3<T>::Zero());
  for (auto [p, t, Jpt] : Jblock.get_blocks()) {
    // Verify assumption that this indeed is a contact Jacobian.
    DRAKE_DEMAND(Jblock.row_start(p) % 3 == 0);
    DRAKE_DEMAND(Jpt.rows() % 3 == 0);
    // ic_start is the first contact point of patch p.
    const int ic_start = Jblock.row_start(p) / 3;
    // k-th contact within patch p.
    for (int k = 0; k < Jpt.rows() / 3; k++) {
      const int ic = ic_start + k;
      const auto& Jkt = Jpt.block(3 * k, 0, 3, Jpt.cols());
      // This effectively computes Jₖₜ⋅M⁻¹⋅Jₖₜᵀ.
      W[ic] += Jkt * M_ldlt[t].solve(Jkt.transpose());
    }
  }

  // Compute Wdiag as the rms norm of k-th diagonal block.
  for (int k = 0; k < nc; ++k) {
    (*Wdiag)[k] = W[k].norm() / 3;
  }
}

template <typename T>
typename SapSolver<T>::PreProcessedData SapSolver<T>::PreProcessData(
    const T& time_step, const SystemDynamicsData<T>& dynamics_data,
    const PointContactData<T>& contact_data) const {
  using std::max;
  using std::min;
  using std::sqrt;

  PreProcessedData data(dynamics_data.num_velocities(),
                        contact_data.num_contacts());

  // Aliases to data.
  const VectorX<T>& mu = contact_data.get_mu();
  const VectorX<T>& phi0 = contact_data.get_phi0();
  const VectorX<T>& stiffness = contact_data.get_stiffness();
  const VectorX<T>& dissipation = contact_data.get_dissipation();

  // Aliases to mutable pre-processed data.
  VectorX<T>& R = data.R;
  VectorX<T>& vhat = data.vhat;
  VectorX<T>& inv_sqrt_M = data.inv_sqrt_M;

  data.time_step = time_step;

  // Store operators as block-sparse matrices.
  dynamics_data.get_A().AssembleMatrix(&data.Mblock);
  contact_data.get_Jc().AssembleMatrix(&data.Jblock);

  // Extract mass matrix's per-tree diagonal blocks.
  // Compute diagonal scaling inv_sqrt_M.
  data.Mt.clear();
  data.Mt.reserve(data.Mblock.num_blocks());
  for (const auto& block : data.Mblock.get_blocks()) {
    const int t1 = std::get<0>(block);
    const int t2 = std::get<1>(block);
    const MatrixX<T>& Mij = std::get<2>(block);
    // We verify the assumption that M is block diagonal.
    DRAKE_DEMAND(t1 == t2);
    // Each block must be squared.
    DRAKE_DEMAND(Mij.rows() == Mij.cols());

    const int nt = Mij.rows();  // Number of DOFs in the tree.
    data.Mt.push_back(Mij);

    const int start = data.Mblock.row_start(t1);
    DRAKE_DEMAND(start == data.Mblock.col_start(t2));
    inv_sqrt_M.template segment(start, nt) =
        Mij.diagonal().cwiseInverse().cwiseSqrt();
  }

  // Computation of a diagonal approximation to the Delassus operator.
  // N.B. This must happen before the computation of the regularization R below.
  const int nc = phi0.size();
  CalcDelassusDiagonalApproximation(nc, data.Mt, data.Jblock, &data.Wdiag);

  // We use the Delassus scaling computed above to estimate regularization
  // parameters in the matrix R.
  const VectorX<T>& Wdiag = data.Wdiag;
  const double beta = parameters_.beta;
  const double sigma = parameters_.sigma;

  // Regularization for near-rigid bodies is computed as Rₙ = α²/(4π²)⋅Wᵢ when
  // the contact frequency ωₙ is below the limit ωₙ⋅dt ≤ 2π. That is, the period
  // is Tₙ = α⋅dt.
  const T beta_factor = beta * beta / (4.0 * M_PI * M_PI);
  for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    const T& k = stiffness(ic);
    const T& c = dissipation(ic);
    DRAKE_DEMAND(k > 0 && c >= 0);
    const T& Wi = Wdiag(ic);
    const T taud = c / k;  // Damping time scale.
    const T Rn =
        max(beta_factor * Wi, 1.0 / (time_step * k * (time_step + taud)));
    const T Rt = sigma * Wi;
    R.template segment<3>(ic3) = Vector3<T>(Rt, Rt, Rn);

    // Stabilization velocity.
    const T vn_hat = -phi0(ic) / (time_step + taud);
    vhat.template segment<3>(ic3) = Vector3<T>(0, 0, vn_hat);
  }

  data.Rinv = R.cwiseInverse();
  data.v_star = dynamics_data.get_v_star();
  data.mu = mu;
  data.Mblock.Multiply(data.v_star, &data.p_star);

  return data;
}

template <typename T>
void SapSolver<T>::CalcAnalyticalInverseDynamics(
    double soft_norm_tolerance, const VectorX<T>& vc, VectorX<T>* gamma,
    std::vector<Matrix3<T>>* dgamma_dy, VectorX<int>* regions) const {
  const int nc = data_.nc;
  const int nc3 = 3 * nc;
  DRAKE_DEMAND(vc.size() == nc3);
  DRAKE_DEMAND(gamma->size() == nc3);

  // Computes the "soft norm" ‖x‖ₛ defined by ‖x‖ₛ² = ‖x‖² + ε², where
  // ε = soft_tolerance.
  auto soft_norm = [](const Eigen::Ref<const VectorX<T>>& x,
                      double soft_tolerance) -> T {
    using std::sqrt;
    return sqrt(x.squaredNorm() + soft_tolerance * soft_tolerance);
  };

  // Pre-processed data.
  const auto& R = data_.R;
  const auto& vhat = data_.vhat;

  // Problem data.
  const auto& mu_all = data_.mu;

  if (dgamma_dy != nullptr) DRAKE_DEMAND(regions != nullptr);

  for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    const auto& vc_ic = vc.template segment<3>(ic3);
    const auto& vhat_ic = vhat.template segment<3>(ic3);
    const auto& R_ic = R.template segment<3>(ic3);
    const T& mu = mu_all(ic);
    const T& Rt = R_ic(0);
    const T& Rn = R_ic(2);
    const Vector3<T> y_ic = (vhat_ic - vc_ic).array() / R_ic.array();
    const auto yt = y_ic.template head<2>();
    const T yr = soft_norm(yt, soft_norm_tolerance);
    const T yn = y_ic[2];
    const Vector2<T> that = yt / yr;

    // Analytical projection of y onto the friction cone ℱ using the R norm.
    auto gamma_ic = gamma->template segment<3>(ic3);
    if (dgamma_dy != nullptr) {
      auto& dgamma_dy_ic = (*dgamma_dy)[ic];
      gamma_ic = CalcProjection({mu, Rt, Rn}, y_ic, yr, yn, that,
                                &(*regions)(ic), &dgamma_dy_ic);
    } else {
      int region_ic{-1};
      gamma_ic = CalcProjection({mu, Rt, Rn}, y_ic, yr, yn, that, &region_ic);
    }
  }
}

template <typename T>
void SapSolver<T>::PackContactResults(const PreProcessedData& data,
                                      const VectorX<T>& v, const VectorX<T>& vc,
                                      const VectorX<T>& gamma,
                                      ContactSolverResults<T>* results) const {
  results->Resize(data.nv, data.nc);
  results->v_next = v;
  ExtractNormal(vc, &results->vn);
  ExtractTangent(vc, &results->vt);
  ExtractNormal(gamma, &results->fn);
  ExtractTangent(gamma, &results->ft);
  // N.B. While contact solver works with impulses, results are reported as
  // forces.
  results->fn /= data.time_step;
  results->ft /= data.time_step;
  const auto& Jop = data.Jblock;
  Jop.MultiplyByTranspose(gamma, &results->tau_contact);
  results->tau_contact /= data.time_step;
}

template <typename T>
void SapSolver<T>::CalcScaledMomentumAndScales(
    const PreProcessedData& data, const VectorX<T>& v, const VectorX<T>& gamma,
    T* scaled_momentum_error, T* momentum_scale, T* Ek, T* ellM, T* ellR,
    T* ell, VectorX<T>* v_work1, VectorX<T>* v_work2,
    VectorX<T>* v_work3) const {
  using std::max;

  const int nv = data.nv;
  const auto& v_star = data.v_star;
  const auto& p_star = data.p_star;
  const auto& inv_sqrt_M = data.inv_sqrt_M;
  const auto& R = data.R;
  const auto& M = data.Mblock;
  const auto& J = data.Jblock;

  VectorX<T>& p = *v_work1;
  M.Multiply(v, &p);

  VectorX<T>& j = *v_work2;
  J.MultiplyByTranspose(gamma, &j);

  VectorX<T>& grad_ell = *v_work3;
  grad_ell = p - p_star - j;

  // Energy metrics.
  *Ek = 0.5 * v.dot(p);
  const T Ek_star = 0.5 * v_star.dot(p_star);  // TODO: move to pre-proc data.
  *ellM = *Ek + Ek_star - v.dot(p_star);
  if (*ellM < 0) {
    PRINT_VAR(*ellM);
  }
  DRAKE_DEMAND(*ellM >= 0);
  *ellR = 0.5 * gamma.dot(R.asDiagonal() * gamma);
  *ell = *ellM + *ellR;

  // Scale momentum balance using the mass matrix's Jacobi preconditioner so
  // that all entries have the same units and we can compute a fair error
  // metric.
  grad_ell = inv_sqrt_M.asDiagonal() * grad_ell;
  p = inv_sqrt_M.asDiagonal() * p;
  j = inv_sqrt_M.asDiagonal() * j;

  *scaled_momentum_error = grad_ell.norm();
  *momentum_scale = max(p.norm(), j.norm());
}

template <typename T>
ContactSolverStatus SapSolver<T>::DoSolveWithGuess(
    const PreProcessedData& data, const VectorX<T>& v_guess,
    ContactSolverResults<T>* result) {
  throw std::logic_error("Only T = double is supported.");
}

template <>
ContactSolverStatus SapSolver<double>::DoSolveWithGuess(
    const PreProcessedData& data, const VectorX<double>& v_guess,
    ContactSolverResults<double>* results) {
  using std::abs;
  using std::max;

  const int nv = data_.nv;
  const int nc = data_.nc;
  const int nc3 = 3 * nc;

  if (parameters_.verbosity_level >= 1) PrintProblemSizes();
  if (parameters_.verbosity_level >= 2) PrintJacobianSparsity();

  State state(nv, nc);
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
  UpdateImpulsesCache(state, &cache);

  // Previous iteration state, for error computation and reporting.
  State state_kp = state;

  // Super nodal solver is constructed once per time-step to reuse structure
  // of M and J.
  // TODO: Move solver to the cache, so that the factorization is effectively
  // cached. Think of computing gradients later.
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

    // N.B. This update is important and must be here!
    UpdateImpulsesCache(state, &cache);

    double scaled_momentum_error, momentum_scale;
    {
      double Ek, costM, costR, cost;
      // TODO: Reconciliate. consider not computing costs nor Ek here since we
      // dont use them and for the solver we cache them.
      CalcScaledMomentumAndScales(
          data, state.v(), cache.gamma, &scaled_momentum_error, &momentum_scale,
          &Ek, &costM, &costR, &cost, &v_work1, &v_work2, &v_work3);
    }
    // Note: only update the useful stats. Remove things like mom_rel_max.
    if (scaled_momentum_error <= parameters_.rel_tolerance * momentum_scale) {
      if (parameters_.verbosity_level >= 1)
        PrintConvergedIterationStats(k, state);
      break;
    } else {
      ++num_iterations;  // For statistics, we only count those iterations
                         // that actually do work, i.e. solve a system of
                         // linear equations.
      // Prepare supernodal solver on first iteration only when needed.
      // That is, if converged, avoid this work.
      if (parameters_.use_supernodal_solver && k == 0) {
        solver_ = std::make_unique<conex::SuperNodalSolver>(
            data_.Jblock.block_rows(), data_.Jblock.get_blocks(), data_.Mt);
      }
    }
    state_kp = state;

    UpdateSearchDirectionCache(state, &cache);

    // Perform line-search.
    // N.B. If converged, we allow one last update with alpha = 1.0.
    alpha = 1.0;
    num_ls_iters = 0;  // Count line-search iterations.
    num_ls_iters = PerformBackTrackingLineSearch(state, &alpha);

    // Update state.
    state.mutable_v() += alpha * cache.dv;

    // TODO: refactor into PrintNewtonStats().
    if (parameters_.verbosity_level >= 3) {
      PRINT_VAR(cache.ellM);
      PRINT_VAR(cache.ell);
      PRINT_VAR(cache.dv.norm());
      PRINT_VAR(state_kp.cache().ell);
      PRINT_VAR(alpha);
    }
  }

  if (k == parameters_.max_iterations) return ContactSolverStatus::kFailure;

  PackContactResults(data_, state.v(), cache.vc, cache.gamma, results);

  return ContactSolverStatus::kSuccess;
}

template <typename T>
T SapSolver<T>::CalcLineSearchCost(const State& state_v, const T& alpha,
                                   State* state_alpha) const {
  DRAKE_DEMAND(state_v.cache().valid_line_search_quantities);

  // Data.
  const int nc = data_.nc;
  const auto& R = data_.R;
  const auto& Rinv = data_.Rinv;
  const auto& v_star = data_.v_star;

  // Quantities at state v.
  const auto& dv = state_v.cache().dv;
  const auto& dp = state_v.cache().dp;
  const auto& dvc = state_v.cache().dvc;

  // State at v(alpha).
  state_alpha->mutable_v() = state_v.v() + alpha * dv;  

  // Update velocities and impulses at v(alpha).
  UpdateImpulsesCache(*state_alpha, &state_alpha->mutable_cache());  

  const auto& v = state_alpha->v();
  const auto& gamma = state_alpha->cache().gamma;

  // Cost ellR.
  const T ellR = 0.5 * gamma.dot(R.asDiagonal() * gamma);

  // We can compute ellM in terms of precomputed terms.
  T ellM = state_v.cache().ellM;
  ellM += alpha * dp.dot(state_v.v() - v_star);
  ellM += 0.5 * alpha * alpha * state_v.cache().d2ellM_dalpha2;
  const T ell = ellM + ellR;

  return ell;
}

template <typename T>
int SapSolver<T>::PerformBackTrackingLineSearch(const State& state,
                                                T* alpha_out) const {
  DRAKE_DEMAND(state.cache().gradients_updated);

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
  const auto& Aop = data_.Mblock;
  Aop.Multiply(cache.dv, &cache.dp);  // M * cache.dv;
  cache.d2ellM_dalpha2 = cache.dv.dot(cache.dp);
  cache.valid_line_search_quantities = true;

  // Save dot product between dv and ell_grad_v.
  const T dell_dalpha0 = ell_grad_v0.dot(dv);

  // dℓ/dα(α = 0) is guaranteed to be strictly negative given the the Hessian of
  // the cost is positive definite. Only round-off errors in the factorization
  // of the Hessian for ill-conditioned systems (small regularization) can
  // destroy this property. If so, we abort given that'd mean the model must be
  // revisited.
  DRAKE_DEMAND(dell_dalpha0 < 0);

  T alpha = parameters_.ls_alpha_max;
  State state_aux(state);  // Auxiliary workspace.
  T ell_alpha = CalcLineSearchCost(state, alpha, &state_aux);

  T alpha_prev = alpha;
  T ell_prev = ell_alpha;

  // Record if the previous iteration satisfies the Armijo condition.
  bool satisfies_armijo_prev = ell_alpha < ell0 + c * alpha * dell_dalpha0;

  int num_iters = 0;
  for (int iter = 0; iter < max_iterations; ++iter) {
    ++num_iters;
    alpha *= rho;
    ell_alpha = CalcLineSearchCost(state, alpha, &state_aux);
    const bool satisfies_armijo = ell_alpha < ell0 + c * alpha * dell_dalpha0;
    // std::cout << alpha << " " << ell_alpha - ell0 << " " << satisfies_armijo
    //          << std::endl;
    // Armijo's criteria.
    // Since we know the function is convex, we in addition continue iterating
    // until ell starts increasing.
    if (ell_alpha > ell_prev) {
      if (satisfies_armijo) {
        // TODO: move expensive computation of gradients into this scope since I
        // only need them here!

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

  // TODO: it might be we have a real shapre cost near alpha=0 that the
  // backtracking line search could not resolve and therefore the cost never
  // went up. However, if Armijo's condition is satisfied, we still are game.
  // You should check that here!
  const bool satisfies_armijo = ell_alpha < ell0 + c * alpha * dell_dalpha0;
  if (satisfies_armijo) {
    *alpha_out = alpha;
    return num_iters;
  }

  throw std::runtime_error("Line search reached max iterations.");
  DRAKE_UNREACHABLE();
}

template <typename T>
void SapSolver<T>::CallSupernodalSolver(const State& s, VectorX<T>* dv,
                                        conex::SuperNodalSolver* solver) const {
  auto& cache = s.mutable_cache();

  UpdateCostAndGradientsCache(s, &cache);

  // This call does the actual assembly H = A + J G Jᵀ.
  solver->SetWeightMatrix(cache.G);

  // Factor() overwrites the assembled matrix with its LLT decomposition.
  // We'll count it as part of the linear solver time.
  solver->Factor();
  *dv = -cache.ell_grad_v;  // we solve dv in place.
  solver->SolveInPlace(dv);
}

template <typename T>
void SapSolver<T>::CallDenseSolver(const State& state, VectorX<T>* dv) const {
  const int nv = data_.nv;

  auto& cache = state.mutable_cache();
  UpdateCostAndGradientsCache(state, &cache);

  {
    int nc = data_.nc;
    const auto& A = data_.Mblock;
    const auto& J = data_.Jblock;
    const auto& G = cache.G;    

    MatrixX<T> Jdense(3 * nc, nv);
    Jdense = J.MakeDenseMatrix();
    MatrixX<T> Adense(nv, nv);
    Adense = A.MakeDenseMatrix();

    MatrixX<T> GJ(3 * nc, nv);
    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      const MatrixX<T>& G_ic = G[ic];
      GJ.block(ic3, 0, 3, nv) = G_ic * Jdense.block(ic3, 0, 3, nv);
    }
    cache.ell_hessian_v = Adense + Jdense.transpose() * GJ;
    cache.valid_dense_gradients = true;
  }

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

template <typename T>
void SapSolver<T>::PrintProblemSizes() const {
  const int nv = data_.nv;
  const int nc = data_.nc;
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

template <typename T>
void SapSolver<T>::PrintJacobianSparsity() const {
  for (const auto& [p, t, Jb] : data_.Jblock.get_blocks()) {
    std::cout << fmt::format("(p,t) = ({:d},{:d}). {:d}x{:d}.\n", p, t,
                             Jb.rows(), Jb.cols());
  }
}

template <typename T>
void SapSolver<T>::PrintConvergedIterationStats(int k, const State& s) const {
  const auto& cache = s.cache();
  std::cout << "Iteration converged at: " << k << std::endl;
  std::cout << "ell: " << cache.ell << std::endl;
  PRINT_VAR(cache.vc.norm());
  std::cout << std::string(80, '=') << std::endl;
}

template <typename T>
void SapSolver<T>::UpdateVelocitiesCache(const State& state,
                                         Cache* cache) const {
  if (cache->velocities_updated) return;
  const auto& Jc = data_.Jblock;
  Jc.Multiply(state.v(), &cache->vc);
  cache->velocities_updated = true;
}

template <typename T>
void SapSolver<T>::UpdateImpulsesCache(const State& state, Cache* cache) const {
  if (cache->impulses_updated) return;
  UpdateVelocitiesCache(state, cache);
  CalcAnalyticalInverseDynamics(parameters_.soft_tolerance, cache->vc,
                                &cache->gamma);
  cache->impulses_updated = true;
}

template <typename T>
void SapSolver<T>::UpdateMomentumChangeCache(const State& state,
                                             Cache* cache) const {
  if (cache->momentum_change_updated) return;
  data_.Mblock.Multiply(state.v(), &cache->momentum_change);  // p = A⋅v.
  cache->momentum_change -= data_.p_star;  // = p - p* = A⋅(v−v*).
  cache->momentum_change_updated = true;
}

// Dependencies: impulses_updated, momentum_change_updated.
template <typename T>
void SapSolver<T>::UpdateCostCache(const State& state, Cache* cache) const {
  if (cache->cost_updated) return;
  UpdateImpulsesCache(state, cache);
  UpdateMomentumChangeCache(state, cache);
  const auto& R = data_.R;
  const auto& v_star = data_.v_star;
  const auto& Adv = cache->get_momentum_change();
  const VectorX<T>& v = state.v();
  const VectorX<T>& gamma = cache->get_gamma();
  cache->ellM = 0.5 * Adv.dot(v - v_star);
  cache->ellR = 0.5 * gamma.dot(R.asDiagonal() * gamma);
  cache->ell = cache->ellM + cache->ellR;
  cache->cost_updated = true;
}

// Dependencies: velocities_updated, impulses_updated.
// Updates: impulses_updated, gradients_updated, cost_updated.
template <typename T>
void SapSolver<T>::UpdateCostAndGradientsCache(const State& state,
                                               Cache* cache) const {
  if (cache->gradients_updated) return;
  UpdateMomentumChangeCache(state, cache);
  UpdateVelocitiesCache(state, cache);

  // Update γ(v) and dγ/dy(v).
  // N.B. We update impulses and gradients together so that the we can reuse
  // common terms in the analytical inverse dynamics.
  CalcAnalyticalInverseDynamics(parameters_.soft_tolerance, cache->get_vc(),
                                &cache->gamma, &cache->dgamma_dy,
                                &cache->regions);
  cache->impulses_updated = true;

  UpdateCostCache(state, cache);

  // Update ∇ᵥℓ.
  const VectorX<T>& gamma = cache->get_gamma();
  const VectorX<T>& Adv = cache->get_momentum_change();
  data_.Jblock.MultiplyByTranspose(gamma, &cache->ell_grad_v);  // = Jᵀγ
  cache->ell_grad_v = -cache->ell_grad_v;                       // = -Jᵀγ
  cache->ell_grad_v += Adv;  // = A⋅(v−v*) - Jᵀγ

  // Update G.
  const int nc = data_.nc;
  const auto& R = data_.R;
  const auto& dgamma_dy = cache->dgamma_dy;
  for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    const auto& R_ic = R.template segment<3>(ic3);
    const Vector3<T> Rinv = R_ic.cwiseInverse();
    const Matrix3<T>& dgamma_dy_ic = dgamma_dy[ic];
    MatrixX<T>& G_ic = cache->G[ic];
    G_ic = dgamma_dy_ic * Rinv.asDiagonal();
  }

  cache->gradients_updated = true;
}

template <typename T>
void SapSolver<T>::UpdateSearchDirectionCache(const State& state,
                                              Cache* cache) const {
  if (cache->search_direction_updated) return;

#if 0 
  VectorX<T> dv;       // search direction.
    VectorX<T> dvc;      // Search direction in contact velocities.
    VectorX<T> dp;     // Δp = M⋅Δv
    T d2ellM_dalpha2;  // d2ellM_dalpha2 = Δvᵀ⋅M⋅Δv = Δvᵀ⋅Δp
#endif

  // Update search direction dv.
  // TODO: get rid of CallDenseSolver(). Only here for debbuging.
  if (parameters_.use_supernodal_solver) {
    CallSupernodalSolver(state, &cache->dv, solver_.get());
  } else {
    CallDenseSolver(state, &cache->dv);
  }

  // Update change in contact velocities.
  const auto& Jop = data_.Jblock;
  Jop.Multiply(cache->dv, &cache->dvc);

  data_.Mblock.Multiply(cache->dv, &cache->dp);
  cache->d2ellM_dalpha2 = cache->dv.dot(cache->dp);

  cache->search_direction_updated = true;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::SapSolver<double>;
