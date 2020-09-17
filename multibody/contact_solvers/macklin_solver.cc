#include "drake/multibody/contact_solvers/macklin_solver.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {
namespace contact_solvers {

using Eigen::SparseMatrix;
using Eigen::SparseVector;

template <typename T>
void MacklinSolver<T>::SetPointContactData(
    const PointContactData<T>* data) {
  DRAKE_DEMAND(data != nullptr);
  contact_data_ = data;
}

template <typename T>
void MacklinSolver<T>::SetSystemDynamicsData(const SystemDynamicsData<T>* data) 
{
  DRAKE_DEMAND(data != nullptr);
  dynamics_data_ = data;
}

template <typename T>
void MacklinSolver<T>::PreProcessData() {
  const int nc = num_contacts();
  const int nv = num_velocities();
  state_.Resize(nv, nc);
  pre_proc_data_.Resize(nv, nc);
  scratch_workspace_.Resize(nv, nc, 64);

  if (nc != 0) {
    // Contact velocities when contact forces are zero.
    auto& vc_star = pre_proc_data_.vc_star;
    get_Jc().Multiply(get_v_star(), &vc_star);

    auto& W = pre_proc_data_.W;
    this->FormDelassusOperatorMatrix(get_Jc(), get_Ainv(), get_Jc(), &W);
    PRINT_VARn(W);

    // Compute scaling factors, one per contact.
    auto& Wii_norm = pre_proc_data_.Wii_norm;
    for (int i = 0; i < nc; ++i) {
      // 3x3 diagonal block. It might be singular, but definitely non-zero.
      // That's why we use an rms norm.
      const auto& Wii = W.block(3 * i, 3 * i, 3, 3);
      Wii_norm(i) = Wii.norm() / 3;  // 3 = sqrt(9).
    }
    PRINT_VAR(Wii_norm.transpose());

    // Update vn0, used to compute vn_stab.
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& vc0 = access.xc_sized_vector();
    get_Jc().Multiply(get_v0(), &vc0);
    auto& vn0 = pre_proc_data_.vn0;
    ExtractNormal(vc0, &vn0);

    // Calc stabilization velocity.
    CalcNormalStabilizationVelocity(get_dt(), parameters_.alpha_stab, vn0,
                                    get_phi0(), &pre_proc_data_.vn_stab);
  }
}

template <typename T>
void MacklinSolver<T>::UpdateContactVelocities(const VectorX<T>& v, VectorX<T>* vc,
                                          VectorX<T>* vn,
                                          VectorX<T>* vt) const {
  get_Jc().Multiply(v, vc);
  ExtractNormal(*vc, vn);
  ExtractTangent(*vc, vt);
}

template <typename T>
T MacklinSolver<T>::CalcFischerBurmeister(const T& x, const T& y,
                                     double epsilon_squared, EigenPtr<Vector2<T>> grad_phi) {
  using std::sqrt;
  const T soft_norm = sqrt(x * x + y * y + epsilon_squared);
  if (grad_phi) {
    (*grad_phi)(0) = 1.0 - x / soft_norm;
    (*grad_phi)(1) = 1.0 - y / soft_norm;
  }
  return x + y - soft_norm;
}

// Assumes cache updated with:
//  - vn
template <typename T>
void MacklinSolver<T>::CalcNormalConstraintResidual(
    const VectorX<T>& vn, const VectorX<T>& pi, const T& dt,
    VectorX<T>* cn, VectorX<T>* gn, VectorX<T>* Rn,
    VectorX<T>* DcnDvn, VectorX<T>* DgnDvn, VectorX<T>* DgnDpi) const {
  using std::max;

  const int nc = num_contacts();

  const VectorX<T>& k = get_stiffness();
  const VectorX<T>& d = get_dissipation();
  const double min_stiffness_relative = 1.0e-10;
  const double e2 =
      parameters_.fb_velocity_scale * parameters_.fb_velocity_scale;

  const auto& scaling_factor = pre_proc_data_.Wii_norm;

  const auto& vn_stab = pre_proc_data_.vn_stab;

  for (int i = 0; i < nc; ++i) {
    const T min_stiffness = k(i) * min_stiffness_relative;
    const T damping = 1.0 - d(i) * vn(i);
    const T kv = k(i) * max(0.0, damping) + min_stiffness;
    const T Rn_ic = 1.0 / (dt * dt * kv);
    const T cn_ic = vn(i) - vn_stab(i) + Rn_ic * pi(i);
    if (Rn) (*Rn)(i) = Rn_ic;
    (*cn)(i) = cn_ic;

    const T& ri = scaling_factor(i);  // scales impulse (Ns) to velocity (m/s).
    const T x = cn_ic;
    const T y = ri * pi(i);
    Vector2<T> grad_fFB;
    const T fFB_ic = CalcFischerBurmeister(x, y, e2, &grad_fFB);
    (*gn)(i) = fFB_ic;

    // Calc gradients.
    if (DcnDvn && DgnDvn && DgnDpi) {
      const T H_dampig = damping >= 0 ? 1.0 : 0.0;
      const T dRn_dvn = Rn_ic / kv * k(i) * d(i) * H_dampig;
      const T DcnDvn_ic = 1.0 + dRn_dvn * pi(i);
      (*DcnDvn)(i) = DcnDvn_ic;

      (*DgnDvn)(i) = grad_fFB(0) * DcnDvn_ic;
      (*DgnDpi)(i) = grad_fFB(0) * Rn_ic + grad_fFB(1) * ri;
    }
  }

  VectorX<T> x = *cn;
  VectorX<T> y = pi.array() * scaling_factor.array();
}

template <typename T>
void MacklinSolver<T>::CalcMaxDissipationConstraintResidual(
    const VectorX<T>& vt, const VectorX<T>& lambda, const VectorX<T>& beta,
    const VectorX<T>& pi, VectorX<T>* dlambda, VectorX<T>* dbeta_norm,
    VectorX<T>* W, VectorX<T>* gt) const {
  const int nc = num_contacts();
  DRAKE_DEMAND(vt.size() == 2 * nc);
  DRAKE_DEMAND(beta.size() == 2 * nc);
  DRAKE_DEMAND(pi.size() == nc);
  DRAKE_DEMAND(lambda.size() == nc);
  DRAKE_DEMAND(dlambda->size() == nc);
  DRAKE_DEMAND(dbeta_norm->size() == nc);
  DRAKE_DEMAND(W->size() == nc);
  DRAKE_DEMAND(gt->size() == 2 * nc);

  const double vs = parameters_.stiction_tolerance;
  const auto& mu = get_mu();
  const auto& r = pre_proc_data_.Wii_norm;

  const auto limit_close_to_zero = [](const T& x) {
    constexpr double kSmallestValue = 1.0e-14;
    using std::min;
    using std::max;
    const T y = x >= 0 ? max(kSmallestValue, x) : min(-kSmallestValue, x);
    return y;
  };

  const double e2 =
      parameters_.fb_velocity_scale * parameters_.fb_velocity_scale;

  for (int i = 0; i < nc; ++i) {
    const T x = lambda(i) - vs;
    const Vector2<T> beta_i = beta.template segment<2>(2 * i);
    const T beta_norm = beta_i.norm();
    const T gamma = mu(i) * pi(i) - beta_norm;
    const T y = r(i) * gamma;

    Vector2<T> grad_phi;
    const T phi = CalcFischerBurmeister(x, y, e2, &grad_phi);

    // Fischer: norm_grad_phi >= 3 - 2*sqrt(2) > 0 always.
    const T norm_grad_phi = grad_phi.norm();

    // My approximation:
    //const Vector2<T> dz = -phi / norm_grad_phi * grad_phi;

    // Nvidia's approximation.
    const Vector2<T> dz(-phi, -phi);  // as if grad_phi = 1/sqrt(2) * (1, 1).

    (*dlambda)(i) = dz(0);
    (*dbeta_norm)(i) = -dz(1) / r(i);

    const T num = lambda(i) + (*dlambda)(i);
    const T den = limit_close_to_zero(beta_norm + (*dbeta_norm)(i));
    
    const T W_i = num / den;
    (*W)(i) = W_i;    

    const Vector2<T> vt_i = vt.template segment<2>(2 * i);
    gt->template segment<2>(2 * i) = vt_i + W_i * beta_i;
  }
}

template <typename T>
void MacklinSolver<T>::CalcNormalStabilizationVelocity(const T& dt,
                                                  double alpha_stab,
                                                  const VectorX<T>& vn0,
                                                  const VectorX<T>& phi0,
                                                  VectorX<T>* vn_stab) {
  if (alpha_stab < 0) {
    vn_stab->setZero();
    return;
  }
  // As Todorov, use a Baugmarte-like stabilization.
  // Note that at alpha_stab = 1.0, we retrieve the stabilization velocity used
  // by [Anitescu, 2006]. We want alpha_stab = 1.0 to recover the law π = −kϕ
  // linear with the signed distance ϕ.
  const T tau_rigid = alpha_stab * dt;
  const T kd = dt / tau_rigid;
  const T kp = dt / (tau_rigid * tau_rigid);
  (*vn_stab) = vn0 - kd * vn0 - kp * phi0;
}

template <typename T>
T MacklinSolver<T>::CalcLineSearchParameter(
  const State& s_km, const State& s_kp) const {
  return parameters_.relaxation;
}

#if 0
template <typename T>
T MacklinSolver<T>::LimitNormalUpdate(
  const State& s_km, const State& s_kp, int outer_iter) const {
  using std::min;
  using std::sqrt;

  // Validate cache.
  ValidateNormalConstraintsCache(s_km);
  ValidateNormalConstraintsCache(s_kp);

  // Workspace.
  GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
  auto& y_km = access.xn_sized_vector();
  auto& y_kp = access.xn_sized_vector();
  auto& pi_km = access.xn_sized_vector();
  auto& pi_kp = access.xn_sized_vector();    

  // State.
  const T m_vc = s_km.m();
  const auto& v_km = s_km.v();
  const auto& v_kp = s_kp.v();
  const auto& gamma_km = s_km.gamma();
  const auto& gamma_kp = s_kp.gamma();
  ExtractNormal(gamma_km, &pi_km);
  ExtractNormal(gamma_kp, &pi_kp);

  // Pre-processed data.
  const auto& scaling_factor = pre_proc_data_.Wii_norm;

  // TODO: consider placing these in the cache?
  const auto& x_km = s_km.cache().cn;
  y_km = scaling_factor.array() * pi_km.array();
  const auto& x_kp = s_kp.cache().cn; 
  y_kp = scaling_factor.array() * pi_kp.array();
  
  auto& dx = access.xn_sized_vector();
  auto& dy = access.xn_sized_vector();
  dx = x_kp - x_km;
  dy = y_kp - y_km;

  const T m_min = parameters_.m_min_factor * m_vc;

  const double e2 =
      parameters_.fb_velocity_scale * parameters_.fb_velocity_scale;

  // There are three cases:
  //  1) Any ϕᵏ < m_min    
  //  2) All ϕᵏ > m_min but some ϕᵏ⁺¹ < m_min
  //  3) All ϕᵏ > m_min and all ϕᵏ⁺¹ > m_min
  // Case (1) would only happen at initialization, outer_iter = 0.
  // Case (2) happens when the search direction crosses the boundary m_min.
  // Case (3) leads to a feasible state.
  //
  // For case (1) and (2) we found we need LS to improve convergence.
  // Case (2) is special in that we need a root finding to find the point at
  // which we cross the boundary m_min.
  // Therefore we first identify if we have case (2). If so, we perform root
  // finding, the followed by a line search.

  const auto& phi_km = s_km.cache().phi_n;
  const auto& phi_kp = s_kp.cache().phi_n;

  PRINT_VAR(m_min);
  PRINT_VAR(phi_km.transpose());
  PRINT_VAR(phi_kp.transpose());

  const bool case1 = (phi_km.array() < m_min).any();
  if (case1 && outer_iter != 0 && parameters_.limit_to_feasible_values)
    throw std::logic_error("This should only happen on initialization.");

  const bool case2 =
      (phi_km.array() > m_min).all() && (phi_kp.array() < m_min).any();

  PRINT_VAR(case1);
  PRINT_VAR(case2);

  T alpha_min =
      parameters_.relaxation;  // The alpha at which crossing m_min happens.
  if (case2 && parameters_.limit_to_feasible_values) {    
    // Here phi_kp < m_min and phi_km > m_min.
    // Thus we do a 1D NR along dz to find out where for each constraint in this
    // condition. Then we take the minimum alpha.
    for (int i = 0; i < num_contacts(); ++i) {
      T alpha_i = 0.0;
      int iter = 0;
      const double kLimitTolerance = 1.0e-3;  // used in line search.
      // To guarantee that our rough LS leads to phi > m_min within
      // kLimitTolerance, we the solver for phi > m_min * (1+ 2 *
      // kLimitTolerance)
      const T m_safe = m_min * (1.0 + 2.0 * kLimitTolerance);
      Vector2<T> grad_phi;
      const Vector2<T> dz(dx(i), dy(i));
      for (; iter < 10; ++iter) {  // should converge fast.
        const T x_alpha = x_km(i) + alpha_i * dx(i);
        const T y_alpha = y_km(i) + alpha_i * dy(i);
        const T phi_alpha =
            CalcFischerBurmeister(x_alpha, y_alpha, e2, &grad_phi);
        const T dphi_dalpha = grad_phi.dot(dz);
        const T dalpha = (m_safe - phi_alpha) / dphi_dalpha;
        alpha_i += dalpha;
        if (abs(dalpha) < 1.0e-3) {
          break;  // exit with a rough tolerance is ok.
        }
      }

      alpha_min = min(alpha_min, alpha_i);
    }
  }

  PRINT_VAR(alpha_min);

  State s_alpha(s_km);
  auto& Fvc = access.xc_sized_vector();
  auto line_search_cost = [&](const T& alpha) {
    s_alpha.mutable_v() = (1.0 - alpha) * v_km + alpha * v_kp;
    s_alpha.mutable_gamma() = (1.0 - alpha) * gamma_km + alpha * gamma_kp;
    ValidateNormalConstraintsCache(s_alpha);
    const auto& phi_alpha = s_alpha.cache().phi_n;
    const T phi_residual =
        (phi_alpha.array() - m_vc).matrix().squaredNorm() / num_contacts();
    CalcContactVelocitiesResidual(s_alpha, &Fvc);
    const T mom_residual = Fvc.squaredNorm() / (3 * num_contacts());

    // Both have units of velocity and thus can be added.
    return phi_residual + mom_residual;
  };

  // Now we perform a line search in (0, alpha_min) for all three cases.
  const double kLineSearchParameter = 0.8;
  T alpha = alpha_min;  // we limit search to alpha < alpha_min.

  if (parameters_.max_ls_iters > 0) {
    T phi_alpha = line_search_cost(alpha);
    for (int ls_iter = 0; ls_iter < parameters_.max_ls_iters; ++ls_iter) {
      alpha *= kLineSearchParameter;
      T phi_alpha_next = line_search_cost(alpha);
      if (phi_alpha_next > phi_alpha) break;
      phi_alpha = phi_alpha_next;
    }
    alpha /= kLineSearchParameter;  // revert to previous value.
  }

  return alpha;
}
#endif

template <typename T>
ContactSolverResult MacklinSolver<T>::SolveWithGuess(const VectorX<T>& v_guess) {
  PreProcessData();

  State s_guess(num_velocities(), num_contacts());
  s_guess.SetVelocities(v_guess);

  if (num_contacts() == 0) {
    state_.mutable_v() = get_v_star();
    state_.mutable_gamma().setZero();
    // Even when for no contact lambda has no meaning, we set it to something
    // sensible, the minimum value lambda can have.
    state_.mutable_lambda().setZero();
    stats_ = {};
    return ContactSolverResult::kSuccess;
  }

  // Workspace.
  GrantScratchWorkspaceAccess<T> access(scratch_workspace_);

  // Update vc so that we can use it to estimate a velocity scale.
  //const auto& vc = EvalVc(s_guess);
  auto& vc_star = pre_proc_data_.vc_star;
  //const T m0 = EstimateVelocityScale(vc, vc_star);

  // Estimate a value of pi such that r * pi = m0.
  // To estimate an initial guess for pi, we essentially start from:
  //   vc = W * gamma + vc_star
  // and simplify it to:
  //   vn = r * pi + vn_star, r = ‖Wᵢᵢ‖ᵣₘₛ
  // With the goal of being able to estimate pi by a proper dimensional scaling
  // of vn_star.
  // Setting vn = 0 and enforcing pi > 0, we get:
  //   pi = max(0, -vn_star / r)
  auto& vn_star = access.xn_sized_vector();
  ExtractNormal(vc_star, &vn_star);
  const auto& r = pre_proc_data_.Wii_norm;
  auto& pi = access.xn_sized_vector();
  pi = (-vn_star.array() / r.array()).max(T(0.0));

  // TODO(amcastro-tri): Estimate friction forces.
  // We could use: β = −μπ vₜ/‖vₜ‖ₛ, with ‖vₜ‖ₛ the "soft" norm of vt.
  auto& beta = access.xt_sized_vector();
  beta.setZero();

  // Set the initial guess for gamma.
  MergeNormalAndTangent(pi, beta, &s_guess.mutable_gamma());

  // Print initial conditions.
  PRINT_VAR(EvalVt(s_guess).transpose());
  PRINT_VAR(EvalVn(s_guess).transpose());
  PRINT_VAR(EvalVc(s_guess).transpose());
  PRINT_VAR(pi.transpose());
  PRINT_VAR(s_guess.gamma().transpose());

  return SolveWithGuess(s_guess);
}

template <typename T>
ContactSolverResult MacklinSolver<T>::SolveWithGuess(const State& state_guess) {
  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;

  const int nv = num_velocities();
  const int nc = num_contacts();  

  GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
  auto& dv = access.v_sized_vector();
  auto& xv_aux = access.v_sized_vector();
  auto& dpi = access.xn_sized_vector();
  auto& dbeta = access.xt_sized_vector();
  auto& dgamma = access.xc_sized_vector();
  auto& dvc = access.xc_sized_vector();

  state_ = state_guess;
  stats_ = {};

  if (num_contacts() == 0) {
    state_.mutable_v() = get_v_star();
    return ContactSolverResult::kSuccess;
  }

  const auto& scaling_factor = pre_proc_data_.Wii_norm;  

  // Initialize lambda. Upon convergence, lambda is the magnitude of the
  // tangential velocity at the i-th conctact. Therefore it does make sense to
  // initialize it using the magnitude of the first predicted tangential
  // velocity.
  auto& vt = EvalVt(state_);
  auto& lambda = state_.mutable_lambda();
  for (int i = 0; i < nc; ++i) {
    lambda(i) = vt.template segment<2>(2 * i).norm();
  }

  // Maybe place these in the cache?
  //MatrixX<T> G(3 * nc, nv);
  SparseMatrix<T> Del(3 * nc, 3 * nc);  // Delassus op.
  SparseMatrix<T> A(3 * nc, 3 * nc);
  VectorX<T> C(3 * nc);
  VectorX<T>& Fv = access.v_sized_vector();
  VectorX<T>& Fgamma = access.xc_sized_vector();
  VectorX<T>& gc = access.xc_sized_vector();

  // Auxiliary state to store update without iteration limit.
  // That is, state_kp.v = state_.v + dv, similarly for gamma.
  State state_kp(state_);

  // Sice the pattern of A is the same as that of N, we analyze it here first.
  // Make sure diagonal entries have non-zeros (it might be N diagonals have
  // zero entries).
  const auto& W = pre_proc_data_.W;
  A = VectorX<T>::Ones(3 * nc).asDiagonal();
  A += W;
  Eigen::SparseLU<SparseMatrix<T>> solver;
  solver.analyzePattern(A);

  int num_inner = 0;
  for (int iter = 0; iter < parameters_.max_iters; ++iter) {
    ++num_inner;
    PRINT_VAR("---------------------------------------------\n");
    PRINT_VAR(iter);

    // Build contact constraint residual gc, such that for the ic-ith contact
    // point we have gc_ic = [gt_ic(0); gt_ic(1); gn_ic]
    const VectorX<T>& gn = Eval_gn(state_);
    const VectorX<T>& gt = Eval_gt(state_);
    MergeNormalAndTangent(gn, gt, &gc);

    // Build diagonal matrix C, st. C_ic = [Wmdp_ic; Wmdp_ic; S_ic]
    const VectorX<T>& Wmdp = Eval_Wmdp(state_);
    const VectorX<T>& DgnDpi = Eval_DgnDpi(state_);
    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      C.template segment<2>(ic3) = Vector2<T>(Wmdp(ic), Wmdp(ic));
      C(ic3 + 2) = DgnDpi(ic);
    }

    // NOTE! Maybe with a CG solver we'd only need the operator form of A,
    // which I believe I can compute using operator forms of Jn, Jt since G =
    // diag(DgnDvn) * Jn, and similarly for Jt. Assemble Schur complement of
    // J = [M -G^T]
    //     [G   C ]
    // C = [Wmdp 0]
    //     [   0 DgnDpi]

    // TODO(amcastro-tri): Implement operator form of A.
    const SparseMatrix<T> Wtilde = EvalWtilde(state_);

    // This line assumes diagonal entries were allocated. See:
    // https://forum.kde.org/viewtopic.php?f=74&t=133686&p=359423&hilit=sparse+diagonal#p359423
    // Otherwise we'll get an error at runtime.
    A = C.asDiagonal();
    A += Wtilde;  // A = Wtilde + C

    PRINT_VARn(MatrixX<T>(Wtilde));
    PRINT_VAR(C.transpose());
    PRINT_VAR(DgnDpi.transpose());
    PRINT_VAR(Wmdp.transpose());
    PRINT_VARn(MatrixX<T>(A));

    CalcVelocitiesResidual(state_, &Fv);

    // We will solve the for the impulses using the Schur-Complement of the
    // system of equations on v and gamma. This leads to:
    // A * dgamma = Gc * Fv - gc    

    // Compute the residual for the impulses, Fgamma.
    GcOperator Gc(this, &state_);  // Gc = DgcDv = DgcDvc * Jc
    Gc.Multiply(Fv, &Fgamma);      // Fgamma = Gc * Fv
    Fgamma -= gc;                  // Fgamma = Gc * Fv - gc;

    // Pattern was analyzed at the very top with A.analyzePattern(). Here the
    // actual numerical factorization is performed.
    solver.factorize(A);
    if (solver.info() != Eigen::Success) {
      return ContactSolverResult::kFailure;
    }
    dgamma = solver.solve(Fgamma);
    if (solver.info() != Eigen::Success) {
      return ContactSolverResult::kFailure;
    }

    // Update dv:
    const LinearOperator<T>& Jv =
        parameters_.macklin_jacobian ? Gc : get_Jc();
    Jv.MultiplyByTranspose(dgamma, &xv_aux);  // xv_aux = Jvᵀ * dgamma
    get_Ainv().Multiply(xv_aux, &dv);         // dv = A⁻¹ * Jvᵀ * dgamma
    dv -= Fv;                                 // dv = -Fv + A⁻¹ * Jvᵀ * dgamma

    PRINT_VAR(Fgamma.transpose());
    PRINT_VAR(Fv.transpose());
    PRINT_VAR(dgamma.transpose());
    PRINT_VAR(dv.transpose());

    // Store v += dv, gamma += dgamma and lambda += dlambda in state_kp.
    state_kp = state_;
    state_kp.mutable_v() += dv;
    state_kp.mutable_gamma() += dgamma;
    state_kp.mutable_lambda() += state_.cache().dlambda;

    // Line search.
    const T alpha = CalcLineSearchParameter(state_, state_kp);

    // Perform the actual solver state update.
    state_.mutable_v() += alpha * dv;
    state_.mutable_gamma() += alpha * dgamma;
    state_.mutable_lambda() += alpha * state_.cache().dlambda;

    PRINT_VAR(state_.v().transpose());
    PRINT_VAR(state_.gamma().transpose());    

    ErrorMetrics errors;
    const bool converged = CheckConvergenceCriteria(state_, &errors);
    // Update stats.
    stats_.iterations++;
    stats_.iteration_errors.push_back(errors);

    if (converged) return ContactSolverResult::kSuccess;;
  }  // iter
  return ContactSolverResult::kFailure;
}

}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::MacklinSolver)
