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
      // dPdy = |dgt_dyt dgt_dyn| |dgn_dyt dgn_dyn| where dgt_dyt ∈ ℝ²ˣ²,
      //        dgt_dyn ∈ ℝ², dgn_dyt ∈ ℝ²ˣ¹ and dgn_dyn ∈ ℝ.
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

  // Extract mass matrix's per-tree diagonal blocks. Compute diagonal scaling
  // inv_sqrt_M.
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

  // Computation of a diagonal approximation to the Delassus operator. N.B. This
  // must happen before the computation of the regularization R below.
  const int nc = phi0.size();
  CalcDelassusDiagonalApproximation(nc, data.Mt, data.Jblock, &data.Wdiag);

  // We use the Delassus scaling computed above to estimate regularization
  // parameters in the matrix R.
  const VectorX<T>& Wdiag = data.Wdiag;
  const double beta = parameters_.beta;
  const double sigma = parameters_.sigma;

  // Rigid approximation contant: Rₙ = β²/(4π²)⋅Wᵢ when the contact frequency ωₙ
  // is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
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

  // Computes the "soft norm" ‖x‖ₛ defined by ‖x‖ₛ² = ‖x‖² + ε², where ε =
  // soft_tolerance.
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
void SapSolver<T>::CalcStoppingCriteriaResidual(const State& state,
                                                T* momentum_residual,
                                                T* momentum_scale) const {
  using std::max;
  const auto& inv_sqrt_M = data_.inv_sqrt_M;
  const VectorX<T>& p = state.cache().momentum_cache().p;
  const VectorX<T>& j = state.cache().momentum_cache().j;
  const VectorX<T>& ell_grad = state.cache().gradients_cache().ell_grad_v;

  // Scale generalized momentum quantities using inv_sqrt_M so that all entries
  // have the same units and we can weigh them equally.
  const VectorX<T> ell_grad_tilde = inv_sqrt_M.asDiagonal() * ell_grad;
  const VectorX<T> p_tilde = inv_sqrt_M.asDiagonal() * p;
  const VectorX<T> j_tilde = inv_sqrt_M.asDiagonal() * j;

  *momentum_residual = ell_grad_tilde.norm();
  *momentum_scale = max(p_tilde.norm(), j_tilde.norm());
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

  state_.Resize(nv, nc);
  State& state = state_;
  stats_.Reset();

  state.mutable_v() = v_guess;
  auto& cache = state.mutable_cache();

  // We perform the first update here so that we can evaluate the stopping
  // criteria before we need an expensive factorization. If the state
  // satisfies the stopping criteria we exit before a factorization is
  // performed. In particular, if the initial guess satisfies the stopping
  // criteria, the solver exits without performing a single factorization.
  // Updating all cache entries here has the advantage that the first
  // computation of the impulses is performed along with the computation of
  // its gradients. Computing γ and dγ/dy can be performed in a single pass.
  UpdateCostAndGradientsCache(state, &cache);
  double ell_previous = cache.cost_cache().ell;

  // Super nodal solver is constructed once per time-step to reuse structure of
  // M and J. TODO: Move solver to the cache, so that the factorization is
  // effectively cached. Think of computing gradients later.
  std::unique_ptr<conex::SuperNodalSolver> solver;

  // Start Newton iterations.
  int k = 0;
  for (; k < parameters_.max_iterations; ++k) {
    if (parameters_.verbosity_level >= 3) {
      std::cout << std::string(80, '=') << std::endl;
      std::cout << std::string(80, '=') << std::endl;
      std::cout << "Iteration: " << k << std::endl;
    }

    // We first verify the stopping criteria. If satisfied, we skip expensive
    // factorizations.
    double momentum_residual, momentum_scale;
    CalcStoppingCriteriaResidual(state, &momentum_residual, &momentum_scale);
    
    if (momentum_residual <= parameters_.rel_tolerance * momentum_scale) {
      if (parameters_.verbosity_level >= 1)
        PrintConvergedIterationStats(k, state);
      break;
    } else {
      // Prepare supernodal solver on the first iteration it is needed. If the
      // stopping criteria is satisfied at k = 0 (good guess), then we skip the
      // expensive instantiation of the solver.
      if (parameters_.use_supernodal_solver && k == 0) {
        solver_ = std::make_unique<conex::SuperNodalSolver>(
            data_.Jblock.block_rows(), data_.Jblock.get_blocks(), data_.Mt);
      }
    }

    // This is the most expensive update: it performs the factorization of H to
    // solve for the search direction dv.
    UpdateSearchDirectionCache(state, &cache);

    int ls_iters;
    const double alpha = PerformBackTrackingLineSearch(state, &ls_iters);
    stats_.num_line_search_iters += ls_iters;

    // TODO: refactor into PrintNewtonStats().
    if (parameters_.verbosity_level >= 3) {
      PRINT_VAR(cache.cost_cache().ellM);
      PRINT_VAR(cache.cost_cache().ell);
      PRINT_VAR(cache.search_direction_cache().dv.norm());
      PRINT_VAR(alpha);
    }

    // Update state.
    state.mutable_v() += alpha * cache.search_direction_cache().dv;

    // We update the cost here so that we can verify it is decreasing on each
    // iteration.
    // This call also has the effect of updating the state as needed to verify
    // the stopping criteria at the begining of the next iteration.
    UpdateCostAndGradientsCache(state, &cache);
    DRAKE_DEMAND(state.cache().cost_cache().ell < ell_previous);
    ell_previous = state.cache().cost_cache().ell;    
  }

  if (k == parameters_.max_iterations) return ContactSolverStatus::kFailure;

  PackContactResults(data_, state.v(), cache.vc(), cache.gamma(), results);

  // N.B. If the stopping criteria is satisfied for k = 0, the solver is not
  // even instantiated and no factorizations are performed (the expensive part
  // of the computation). We report zero number of iterations.
  stats_.num_iters = k;

  return ContactSolverStatus::kSuccess;
}

template <typename T>
T SapSolver<T>::CalcLineSearchCost(const State& state_v, const T& alpha,
                                   State* state_alpha) const {
  // Data.
  const int nc = data_.nc;
  const auto& R = data_.R;
  const auto& Rinv = data_.Rinv;
  const auto& v_star = data_.v_star;

  // Quantities at state v.
  const auto& search_direction_cache = state_v.cache().search_direction_cache();
  const auto& dv = search_direction_cache.dv;
  const auto& dp = search_direction_cache.dp;
  const auto& dvc = search_direction_cache.dvc;
  const T& d2ellM_dalpha2 = search_direction_cache.d2ellM_dalpha2;

  // State at v(alpha).
  state_alpha->mutable_v() = state_v.v() + alpha * dv;

  // Update velocities and impulses at v(alpha).
  UpdateImpulsesCache(*state_alpha, &state_alpha->mutable_cache());

  const auto& v = state_alpha->v();
  const auto& gamma = state_alpha->cache().gamma();

  // Cost ellR.
  const T ellR = 0.5 * gamma.dot(R.asDiagonal() * gamma);

  // We can compute ellM in terms of precomputed terms.
  T ellM = state_v.cache().cost_cache().ellM;
  ellM += alpha * dp.dot(state_v.v() - v_star);
  ellM += 0.5 * alpha * alpha * d2ellM_dalpha2;
  const T ell = ellM + ellR;

  return ell;
}

template <typename T>
T SapSolver<T>::PerformBackTrackingLineSearch(const State& state,
                                              int* num_iterations) const {
  // Quantities at alpha = 0.
  const T ell0 = state.cache().cost_cache().ell;
  const auto& ell_grad_v0 = state.cache().gradients_cache().ell_grad_v;

  // Search direction.
  const VectorX<T>& dv = state.cache().search_direction_cache().dv;

  // Parameters.
  const double rho = parameters_.ls_rho;
  const double c = parameters_.ls_c;
  const int max_iterations = parameters_.ls_max_iterations;

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
    //          << std::endl; Armijo's criteria. Since we know the function is
    //          convex, we in addition continue iterating until ell starts
    //          increasing.
    if (ell_alpha > ell_prev) {
      if (satisfies_armijo) {
        // TODO: move expensive computation of gradients into this scope since I
        // only need them here!

        // We don't go back one because we do know that the current alpha
        // satisfies the Armijo condition. If the previous iterate satisfies the
        // Armijo condition, it is better since ell_prev < ell_alpha.
        if (satisfies_armijo_prev) alpha /= rho;
        // value.
        *num_iterations = num_iters;
        return alpha;
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
    *num_iterations = num_iters;
    return alpha;
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
  solver->SetWeightMatrix(cache.gradients_cache().G);

  // Factor() overwrites the assembled matrix with its LLT decomposition. We'll
  // count it as part of the linear solver time.
  solver->Factor();
  *dv = -cache.gradients_cache().ell_grad_v;  // we solve dv in place.
  solver->SolveInPlace(dv);
}

template <typename T>
void SapSolver<T>::CallDenseSolver(const State& state, VectorX<T>* dv) const {
  const int nv = data_.nv;

  auto& cache = state.mutable_cache();
  UpdateCostAndGradientsCache(state, &cache);

  MatrixX<T> H(nv, nv);
  {
    int nc = data_.nc;
    const auto& A = data_.Mblock;
    const auto& J = data_.Jblock;
    const auto& G = cache.gradients_cache().G;

    MatrixX<T> Jdense(3 * nc, nv);
    Jdense = J.MakeDenseMatrix();
    MatrixX<T> Adense(nv, nv);
    Adense = A.MakeDenseMatrix();

    MatrixX<T> GJ(3 * nc, nv);
    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      const MatrixX<T>& G_ic = G[ic];
      GJ.block(ic3, 0, 3, nv) = G_ic * Jdense.block(ic3, 0, 3, nv);
    }
    H = Adense + Jdense.transpose() * GJ;
  }

  // Factorize Hessian.
  Eigen::LDLT<MatrixXd> Hldlt(H);
  if (Hldlt.info() != Eigen::Success) {
    // return ContactSolverStatus::kFailure;
    throw std::runtime_error("LDLT solver failed.");
  }

  // Compute search direction.
  const VectorX<T> rhs = -cache.gradients_cache().ell_grad_v;
  *dv = Hldlt.solve(rhs);
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
  std::cout << "ell: " << cache.cost_cache().ell << std::endl;
  PRINT_VAR(cache.vc().norm());
  std::cout << std::string(80, '=') << std::endl;
}

template <typename T>
void SapSolver<T>::UpdateVelocitiesCache(const State& state,
                                         Cache* cache) const {
  if (cache->valid_velocities_cache()) return;
  auto& velocities_cache = cache->mutable_velocities_cache();
  const auto& Jc = data_.Jblock;
  Jc.Multiply(state.v(), &velocities_cache.vc);
  velocities_cache.valid = true;
}

template <typename T>
void SapSolver<T>::UpdateImpulsesCache(const State& state, Cache* cache) const {
  if (cache->valid_impulses_cache()) return;
  UpdateVelocitiesCache(state, cache);
  auto& impulses_cache = cache->mutable_impulses_cache();
  CalcAnalyticalInverseDynamics(parameters_.soft_tolerance, cache->vc(),
                                &impulses_cache.gamma);
  ++stats_.num_impulses_cache_updates;
  impulses_cache.valid = true;
}

template <typename T>
void SapSolver<T>::UpdateMomentumCache(const State& state, Cache* cache) const {
  if (cache->valid_momentum_cache()) return;
  UpdateImpulsesCache(state, cache);
  auto& momentum_cache = cache->mutable_momentum_cache();
  data_.Mblock.Multiply(state.v(), &momentum_cache.p);  // p = A⋅v.
  data_.Jblock.MultiplyByTranspose(state.cache().gamma(), &momentum_cache.j);
  // = p - p* = A⋅(v−v*).
  momentum_cache.momentum_change = momentum_cache.p - data_.p_star;
  momentum_cache.valid = true;
}

// Dependencies: impulses_updated, momentum_change_updated.
template <typename T>
void SapSolver<T>::UpdateCostCache(const State& state, Cache* cache) const {
  if (cache->valid_cost_cache()) return;
  UpdateImpulsesCache(state, cache);
  UpdateMomentumCache(state, cache);
  const auto& R = data_.R;
  const auto& v_star = data_.v_star;
  const auto& Adv = cache->momentum_cache().momentum_change;
  const VectorX<T>& v = state.v();
  const VectorX<T>& gamma = cache->gamma();
  auto& cost_cache = cache->mutable_cost_cache();
  cost_cache.ellM = 0.5 * Adv.dot(v - v_star);
  cost_cache.ellR = 0.5 * gamma.dot(R.asDiagonal() * gamma);
  cost_cache.ell = cost_cache.ellM + cost_cache.ellR;
  cost_cache.valid = true;
}

// Dependencies: velocities_updated, impulses_updated. Updates:
// impulses_updated, gradients_updated, cost_updated.
template <typename T>
void SapSolver<T>::UpdateCostAndGradientsCache(const State& state,
                                               Cache* cache) const {
  if (cache->valid_gradients_cache()) return;

  // Update γ(v) and dγ/dy(v). N.B. We update impulses and gradients together so
  // that the we can reuse common terms in the analytical inverse dynamics. N.B.
  // We make this update before updating the momentum or cost cache so that
  // impulses are valid for them. Do not swap the order.
  auto& impulses_cache = cache->mutable_impulses_cache();
  auto& gradients_cache = cache->mutable_gradients_cache();
  UpdateVelocitiesCache(state, cache);
  CalcAnalyticalInverseDynamics(
      parameters_.soft_tolerance, cache->vc(), &impulses_cache.gamma,
      &gradients_cache.dgamma_dy, &gradients_cache.regions);
  ++stats_.num_impulses_cache_updates;
  impulses_cache.valid = true;

  // N.B. Since impulses were updated above along with dγ/dy, these updates will
  // not need to recompute impulses.
  UpdateMomentumCache(state, cache);
  UpdateCostCache(state, cache);

  // Update ∇ᵥℓ.
  const VectorX<T>& gamma = cache->gamma();
  const VectorX<T>& Adv = cache->momentum_cache().momentum_change;
  data_.Jblock.MultiplyByTranspose(gamma,
                                   &gradients_cache.ell_grad_v);  // = Jᵀγ
  gradients_cache.ell_grad_v = -gradients_cache.ell_grad_v;       // = -Jᵀγ
  gradients_cache.ell_grad_v += Adv;  // = A⋅(v−v*) - Jᵀγ

  // Update G.
  const int nc = data_.nc;
  const auto& R = data_.R;
  const auto& dgamma_dy = gradients_cache.dgamma_dy;
  for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    const auto& R_ic = R.template segment<3>(ic3);
    const Vector3<T> Rinv = R_ic.cwiseInverse();
    const Matrix3<T>& dgamma_dy_ic = dgamma_dy[ic];
    MatrixX<T>& G_ic = gradients_cache.G[ic];
    G_ic = dgamma_dy_ic * Rinv.asDiagonal();
  }

  ++stats_.num_gradients_cache_updates;
  gradients_cache.valid = true;
}

template <typename T>
void SapSolver<T>::UpdateSearchDirectionCache(const State& state,
                                              Cache* cache) const {
  if (cache->valid_search_direction_cache()) return;

  auto& search_direction_cache = cache->mutable_search_direction_cache();

  // Update search direction dv. TODO: get rid of CallDenseSolver(). Only here
  // for debbuging.
  if (parameters_.use_supernodal_solver) {
    CallSupernodalSolver(state, &search_direction_cache.dv, solver_.get());
  } else {
    CallDenseSolver(state, &search_direction_cache.dv);
  }

  // Update Δp, Δvc and d²ellM/dα².
  const auto& Jop = data_.Jblock;
  Jop.Multiply(search_direction_cache.dv, &search_direction_cache.dvc);
  data_.Mblock.Multiply(search_direction_cache.dv, &search_direction_cache.dp);
  search_direction_cache.d2ellM_dalpha2 =
      search_direction_cache.dv.dot(search_direction_cache.dp);

  search_direction_cache.valid = true;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::SapSolver<double>;
