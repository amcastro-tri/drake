#include "drake/multibody/solvers/fischer_burmeister_solver.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {
namespace solvers {

using Eigen::SparseMatrix;
using Eigen::SparseVector;


template <typename T>
void FBSolver<T>::SetSystemDynamicsData(const SystemDynamicsData<T>* data) 
{
  DRAKE_DEMAND(data != nullptr);
  dynamics_data_ = data;
  const int nc = num_contacts();
  const int nv = num_velocities();
  state_.Resize(nv, nc);

  GrantScratchWorkspaceAccess access(scratch_workspace_);
  auto& vc0 = access.xc_sized_vector();
  get_Jc().Multiply(get_v0(), &vc0);
  vn0_.resize(nc);
  ExtractNormal(vc0, &vn0_);

#if 0
  const T& dt = data_->dt();
  const auto& tau = data_->get_tau();
  v_star_.resize(nv);
  MultiplyByMinv(tau, &v_star_);  // v_star_ = M⁻¹⋅tau
  v_star_ = v0 + dt * v_star_;
#endif

  N_.resize(3 * nc, 3 * nc);
  FormDelassusOperatorMatrix(get_Jc(), get_Minv(), get_JcT(), &N_);

  // Compute scaling factors, one per contact.
  scaling_factor.resize(nc);
  for (int i = 0; i < nc; ++i) {
    // 3x3 diagonal block. It might be singular, but definitely non-zero. That's
    // why we use an rms norm.
    const auto& Nii = N_.block(3 * i, 3 * i, 3, 3);
    scaling_factor(i) = Nii.norm() / 3;  // 3 = sqrt(9).
  }
  PRINT_VAR(scaling_factor.transpose());
}

#if 0
template <typename T>
FBSolver<T>::FBSolver(const ProblemData<T>* data)
    : data_(*data),
      state_(data->num_velocities(), data->num_contacts()),
      scratch_workspace_(data->num_velocities(), data->num_contacts(), 20) {
  const int nc = num_contacts();
  const int nv = num_velocities();

  state_.SetDt(data_->dt());

  // Mass matrix inverse factorization.
  Mi_ = data_->M().ldlt();

  // Form full contact Jacobian.
  Jc_.resize(3 * nc, nv);
  for (int i = 0; i < nc; ++i) {
    Jc_.block(3 * i, 0, 2, nv) = data_->Jt().block(2 * i, 0, 2, nv);
    Jc_.block(3 * i + 2, 0, 1, nv) = data_->Jn().block(i, 0, 1, nv);
  }

  const auto& v0 = data_->get_v0();
  vn0_ = data_->Jn() * v0;

  const T& dt = data_->dt();
  const auto& tau = data_->get_tau();
  v_star_.resize(nv);
  MultiplyByMinv(tau, &v_star_);  // v_star_ = M⁻¹⋅tau
  v_star_ = v0 + dt * v_star_;

  // Figure out a way to do this with sparsity.
  Mi_times_JcT_ = Mi_.solve(Jc_.transpose());
  N_ = Jc_ * Mi_times_JcT_;

  // Compute scaling factors, one per contact.
  scaling_factor.resize(nc);
  for (int i = 0; i < nc; ++i) {
    // 3x3 diagonal block. It might be singular, but definitely non-zero. That's
    // why we use an rms norm.
    const auto& Nii = N_.template block<3, 3>(3 * i, 3 * i);
    scaling_factor(i) = Nii.norm() / 3;  // 3 = sqrt(9).
  }
}
#endif

template <typename T>
bool FBSolver<T>::CheckOuterLoopConvergenceCriteria(const VectorX<T>& vc,
                                           const VectorX<T>& dvc,
                                           double* max_dvc_norm) const {
  DRAKE_DEMAND(vc.size() == 3*num_contacts());
  using std::abs;
  using std::max;
  bool converged = true;
  *max_dvc_norm = 0;
  // Convergence is monitored component wise.
  for (int i = 0; i < num_contacts(); ++i) {
    const T vs = parameters_.stiction_tolerance;
    const T vc_norm = vc.template segment<3>(3 * i).norm();
    const T tol = parameters_.outer_loop_tolerance * max(vs, vc_norm);
    const T dvc_norm = dvc.template segment<3>(3 * i).norm();    
    if (dvc_norm > tol) {
      converged = false;
    }
    *max_dvc_norm = max(*max_dvc_norm, ExtractDoubleOrThrow(dvc_norm));
  }
  return converged;
}

template <typename T>
bool FBSolver<T>::CheckInnerLoopConvergenceCriteria(const VectorX<T>& g,
                                           const T& m,
                                           double* g_max_norm) const {
  DRAKE_DEMAND(g.size() == num_contacts());  // for now only normal.
  *g_max_norm = ExtractDoubleOrThrow(g.template lpNorm<Eigen::Infinity>());
  double m_double = ExtractDoubleOrThrow(m);  // TODO: m should be double.
  bool converged = *g_max_norm < parameters_.inner_loop_tolerance * m_double;
  return converged;
}

template <typename T>
void FBSolver<T>::UpdateContactVelocities(const VectorX<T>& v, VectorX<T>* vc,
                                          VectorX<T>* vn,
                                          VectorX<T>* vt) const {
  get_Jc().Multiply(v, vc);
  ExtractNormal(*vc, vn);
  ExtractTangent(*vc, vt);
}

template <typename T>
T FBSolver<T>::CalcFischerBurmeister(const T& x, const T& y,
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
void FBSolver<T>::CalcNormalConstraintResidual(
    const VectorX<T>& vn, const VectorX<T>& pi, const T& m_vc, const T& dt,
    VectorX<T>* cn, VectorX<T>* phi, VectorX<T>* gpi, VectorX<T>* Rn,
    VectorX<T>* dcn_dvn, VectorX<T>* dgpi_dvn, VectorX<T>* dgpi_dpi) const {
  using std::max;

  const int nc = num_contacts();

  const VectorX<T>& k = get_stiffness();
  const VectorX<T>& d = get_dissipation();
  const double min_stiffness_relative = 1.0e-10;
  const double e2 =
      parameters_.fb_velocity_scale * parameters_.fb_velocity_scale;

  for (int i = 0; i < nc; ++i) {
    const T min_stiffness = k(i) * min_stiffness_relative;
    const T damping = 1.0 - d(i) * vn(i);
    const T kv = k(i) * max(0.0, damping) + min_stiffness;
    const T Rn_ic = 1.0 / (dt * dt * kv);
    const T cn_ic = vn(i) - vn_stab_(i) + Rn_ic * pi(i);
    if (Rn) (*Rn)(i) = Rn_ic;
    (*cn)(i) = cn_ic;

    const T& ri = scaling_factor(i);  // scales impulse (Ns) to velocity (m/s).
    const T x = cn_ic;
    const T y = ri * pi(i);
    Vector2<T> grad_phi;
    const T phi_ic = CalcFischerBurmeister(x, y, e2, &grad_phi);
    (*phi)(i) = phi_ic;
    (*gpi)(i) = phi_ic - m_vc;

    // Calc gradients.
    if (dcn_dvn && dgpi_dvn && dgpi_dpi) {
      const T H_dampig = damping >= 0 ? 1.0 : 0.0;
      const T dRn_dvn = Rn_ic / kv * k(i) * d(i) * H_dampig;
      const T dcn_dvn_ic = 1.0 + dRn_dvn * pi(i);
      (*dcn_dvn)(i) = dcn_dvn_ic;

      (*dgpi_dvn)(i) = grad_phi(0) * dcn_dvn_ic;
      (*dgpi_dpi)(i) = grad_phi(0) * Rn_ic + grad_phi(1) * ri;
    }
  }

  VectorX<T> x = *cn;
  VectorX<T> y = pi.array() * scaling_factor.array();
}

template <typename T>
void FBSolver<T>::CalcMaxDissipationConstraintResidual(
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
  const auto& r = scaling_factor;

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

    PRINT_VAR(beta_i.transpose());
    PRINT_VAR(pi(i));
    PRINT_VAR(gamma);
    PRINT_VAR(lambda(i));

    PRINT_VAR(x);
    PRINT_VAR(y);

    Vector2<T> grad_phi;
    const T phi = CalcFischerBurmeister(x, y, e2, &grad_phi);

    PRINT_VAR(phi);

    // Fischer: norm_grad_phi >= 3 - 2*sqrt(2) > 0 always.
    const T norm_grad_phi = grad_phi.norm();

    // My approximation:
    //const Vector2<T> dz = -phi / norm_grad_phi * grad_phi;

    // Nvidia's approximation.
    const Vector2<T> dz(-phi, -phi);  // as if grad_phi = 1/sqrt(2) * (1, 1).
    PRINT_VAR(grad_phi.transpose());
    PRINT_VAR(norm_grad_phi);
    PRINT_VAR(dz.transpose());
    PRINT_VAR(r(i));

    (*dlambda)(i) = dz(0);
    (*dbeta_norm)(i) = -dz(1) / r(i);

    PRINT_VAR((*dlambda)(i));
    PRINT_VAR((*dbeta_norm)(i));

    const T num = lambda(i) + (*dlambda)(i);
    const T den = limit_close_to_zero(beta_norm + (*dbeta_norm)(i));
    
    const T W_i = num / den;
    (*W)(i) = W_i;    

    PRINT_VAR(W_i);

    const Vector2<T> vt_i = vt.template segment<2>(2 * i);
    gt->template segment<2>(2 * i) = vt_i + W_i * beta_i;

    PRINT_VAR(vt.transpose());
    PRINT_VAR(beta_i.transpose());
    PRINT_VAR(gt->transpose());

  }
}

template <typename T>
void FBSolver<T>::CalcNormalStabilizationVelocity(const T& dt,
                                                  VectorX<T>* vn_stab) {
  vn_stab->resize(num_velocities());
  if (parameters_.alpha_stab < 0) {
    vn_stab->setZero();
    return;
  }
  const VectorX<T>& vn0 = vn0_;
  // As Todorov, use Baugmarte stabilization.
  const T tau_rigid = parameters_.alpha_stab * dt;
  const VectorX<T>& phi0 = get_phi0();
  const T kd = dt / tau_rigid;
  const T kp = dt / (tau_rigid * tau_rigid);
  for (int i = 0; i < num_contacts(); ++i) {
    (*vn_stab)(i) = vn0(i) - kd * vn0(i) - kp * phi0(i);
  }
}

template <typename T>
T FBSolver<T>::EstimateVelocityScale(const VectorX<T>& vc,
                                     const VectorX<T>& vc_star) const {
  const T vc_norm = vc.template lpNorm<Eigen::Infinity>();
  const T vc_star_norm = vc_star.template lpNorm<Eigen::Infinity>();
  const T vc_min = 1000.0 * parameters_.stiction_tolerance;
  using std::max;
  const T v_scale = max(vc_min, max(vc_norm, vc_star_norm));
  PRINT_VAR(vc_norm);
  PRINT_VAR(vc_star_norm)
  PRINT_VAR(vc_min);  
  return v_scale;
}

template <typename T>
T FBSolver<T>::LimitNormalUpdate(
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

template <typename T>
FBSolverResult FBSolver<T>::SolveWithGuess(const T& dt, const VectorX<T>& v_guess) {
  State s_guess(num_velocities(), num_contacts());
  s_guess.SetDt(dt);
  s_guess.SetVelocities(v_guess);

  // Workspace.
  GrantScratchWorkspaceAccess<T> access(scratch_workspace_);

  // Update vc so that we can use it to estimate a velocity scale.
  const auto& vc = EvalVc(s_guess);
  auto& vc_star = access.xc_sized_vector();
  get_Jc().Multiply(v_star_, &vc_star);
  const T m0 = EstimateVelocityScale(vc, vc_star);
  s_guess.SetComplementaritySlackness(m0);

  // Estimate a value of pi such that r * pi = m0.
  auto& pi = access.xn_sized_vector();
  pi = m0 * scaling_factor.cwiseInverse();

  // TODO: Estimate friction forces.
  auto& beta = access.xt_sized_vector();
  beta.setZero();

  // Set the initial guess for gamma.
  auto& gamma = access.xc_sized_vector();
  MergeNormalAndTangent(pi, beta, &gamma);
  s_guess.SetImpulses(gamma);

  // Print initial conditions.
  PRINT_VAR(s_guess.cache().vn.transpose());
  PRINT_VAR(pi.transpose());
  PRINT_VAR(s_guess.gamma().transpose());
  PRINT_VAR(m0);

  return SolveWithGuess(s_guess);
}

template <typename T>
FBSolverResult FBSolver<T>::SolveWithGuess(const State& state_guess) {
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

  const T& dt = state_guess.dt();

  v_star_.resize(nv);
  get_Minv().Multiply(get_tau(), &v_star_);  // v_star_ = Mi * tau
  v_star_ = get_v0() + dt * v_star_;      // v_star_ = v0 + dt * Mi * tau

  // TODO: you might pass data_ and params_ explicitly here to show dependency?
  CalcNormalStabilizationVelocity(dt, &vn_stab_);

  PRINT_VAR(scaling_factor.transpose());

  state_ = state_guess;

  // Initialize lambda.
  ValidateContactVelocitiesCache(state_);
  auto& vt = access.xt_sized_vector();
  ExtractTangent(state_.cache().vc, &vt);
  auto& lambda = state_.mutable_lambda();
  for (int i = 0; i < nc; ++i) {
    lambda(i) = vt.template segment<2>(2 * i).norm();
  }

#if 0
  // TODO: investigate if there's a better state.
  // Given we are doing this "central path" stuff, I'd probably need a guess
  // that I know is well withing the feasible region. 
  // Eg. Compute normal force analytically given the compliance, zero friction,
  // and use either v = v0 or v = v_star as a guess.
  state_.SetDt(data_->dt());
  state_.SetVelocities(v_guess);
#endif

#if 0
  // Update vc so that we can use it to estimate a velocity scale.
  const auto& vc = EvalVc(state_);
  auto& vc_star = access.xc_sized_vector();
  MultiplyByJc(v_star_, &vc_star);
  const T m0 = EstimateVelocityScale(vc, vc_star);
  state_.SetComplementaritySlackness(m0);

  // Estimate a value of pi such that r * pi = m0.
  auto& pi = access.xn_sized_vector();
  pi = m0 * scaling_factor.cwiseInverse();

  // TODO: Estimate friction forces.
  auto& beta = access.xt_sized_vector();
  beta.setZero();

  // Set the initial guess for gamma.
  auto& gamma = access.xc_sized_vector();
  MergeNormalAndTangent(pi, beta, &gamma);
  state_.SetImpulses(gamma);

  // Print initial conditions.
  PRINT_VAR(state_.cache().vn.transpose());
  PRINT_VAR(pi.transpose());
  PRINT_VAR(state_.gamma().transpose());
  PRINT_VAR(m0);
#endif  

  const double m_final = parameters_.complementary_slackness_tolerance *
                         parameters_.stiction_tolerance;

  stats_ = {};

  // Maybe place these in the cache?
  //MatrixX<T> G(3 * nc, nv);
  SparseMatrix<T> Del(3 * nc, 3 * nc);  // Delassus op.
  SparseMatrix<T> A(3 * nc, 3 * nc);
  VectorX<T> C(3 * nc);
  VectorX<T>& Fv = access.v_sized_vector();
  VectorX<T>& Fg = access.xc_sized_vector();
  VectorX<T>& g = access.xc_sized_vector();

  // State of the previous outer iteration.
  State state_km(state_);

  // Auxiliary state to store update without iteration limit.
  // That is, state_kp.v = state_.v + dv, similarly for gamma.
  State state_kp(state_);

  // Sice the pattern of A is the same as that of N, we analyze it here first.
  // Make sure diagonal entries have non-zeros (it might be N diagonals have
  // zero entries).
  A = VectorX<T>::Ones(3 * nc).asDiagonal();
  A += N_;
  Eigen::SparseLU<SparseMatrix<T>> solver;
  solver.analyzePattern(A);

  // "Outer iteration" aka "Centering step" in barrier methods for convex
  // optimization solvers.
  int num_outer = 0;
  for (int outer_iter = 0; outer_iter < parameters_.outer_loop_max_iters;
       ++outer_iter) {
    ++num_outer;
    PRINT_VAR("---------------------------------------------\n");
    PRINT_VAR(outer_iter);
    PRINT_VAR(state_.m());

    // Inner loop seeks to converge to the specified value of m_vs.
    const int inner_loop_max_iters = outer_iter == 0
                                         ? parameters_.initialization_max_iters
                                         : parameters_.inner_loop_max_iters;
    int num_inner = 0;
    for (int inner_iter = 0; inner_iter < inner_loop_max_iters; ++inner_iter) {
      ++num_inner;
      PRINT_VAR("---------------------------------------------\n");
      PRINT_VAR(inner_iter);

      ValidateNormalConstraintsCache(state_);
      const auto& S = state_.cache().dgpi_dpi;
      const auto& gpi = state_.cache().gpi;

      ValidateMaxDissipationConstraintsCache(state_);
      const auto& gt = state_.cache().gt;
      const auto& W = state_.cache().W;

      ValidateContactConstraintsCache(state_);
      const VectorX<T>& DgDvc = state_.cache().DgDvc;

      // Operators Gc and GcT for the current state.
      GcOperator Gc(this, &state_);   // Gc = DgDvc = [Jt, DgnDvn * Jn]
      GcTOperator GcT(this, &state_);

#if 0
      // Build G = [dgpi_dv; Jt]
      G.resize(3 * nc, nv);
      for (int i = 0; i < nc; ++i) {
        // Gt = Jt
        G.block(3 * i, 0, 2, nv) = Jt.block(2 * i, 0, 2, nv);

        // Gn
        G.block(3 * i + 2, 0, 1, nv) = dgpi_dvn(i) * Jn.block(i, 0, 1, nv);
      }      
#endif      
      
      for (int i = 0; i < nc; ++i) {
        // Build g = [gt; gn]
        g.template segment<2>(3 * i) = gt.template segment<2>(2 * i);
        g(3 * i + 2) = gpi(i);

        // Build C = [W 0]
        //           [0 S]
        C.template segment<2>(3 * i) = Vector2<T>(W(i), W(i));
        C(3 * i + 2) = S(i);
      }

      // NOTE! Maybe with a CG solver we'd only need the operator form of A,
      // which I believe I can compute using operator forms of Jn, Jt since G =
      // diag(dgpi_dvn) * Jn, and similarly for Jt. Assemble Schur complement of
      // J = [M -G^T]
      //     [G   C ]
      // C = [W 0]
      //     [0 S]
      //FormDelassusOperatorMatrix(Gc, get_Minv(), GcT, &A);

      //W = DgDvc * N * DgDvc;
      // These two multiplications are O(2*nnz(A)).
      // We perform them explicitly by scanning the non-zeros of A.
      Del = N_;
      for (int k = 0; k < Del.outerSize(); ++k) {
        for (typename SparseMatrix<T>::InnerIterator it(Del, k); it; ++it) {
          const int i = it.row();
          const int j = it.col();
          it.valueRef() *= (DgDvc(i) * DgDvc(j));          
        }
      }

      // This does not compile:
      //A = DgDvc.asDiagonal() * N_;  // A = D * N
      //A *= DgDvc.asDiagonal();      // A = D * N * D = Gc * Mi * GcT
      
      PRINT_VARn(MatrixX<T>(N_));
      PRINT_VAR(DgDvc.transpose());
      PRINT_VARn(Del);            

      // this line assumes diagonal entries were allocated. See:
      // https://forum.kde.org/viewtopic.php?f=74&t=133686&p=359423&hilit=sparse+diagonal#p359423
      // Otherwise we'll get an error at runtime.
      A = C.asDiagonal();
      A += Del;  // A = Del + C

      PRINT_VAR(C.transpose());
      PRINT_VARn(A);

      // Old dense version:
      //const MatrixX<T> Mi_times_GT = Mi_.solve(G.transpose());
     // A = C.asDiagonal();
      //A += G * Mi_times_GT;
      
      PRINT_VAR(S.transpose());
      PRINT_VAR(W.transpose());
      PRINT_VARn(A);

      CalcVelocitiesResidual(state_, &Fv);

      // Assemble multipliers residual. nu = [lambda, gamma = [beta, pi], mu]
      Gc.Multiply(Fv, &Fg);  // Fg = Gc * Fv
      Fg -= g;               // Fg = Gc * Fv - g;

      // Solve for change in the multipliers.
      // We expect an SPD system here.
      //Eigen::LDLT<MatrixX<T>> Aldlt(A);
      //if (Aldlt.info() != Eigen::Success) {
      //  return FBSolverResult::kLinearSolverFailed;
      //}
      //dgamma = Aldlt.solve(Fg);
      
      // Pattern was analyzed at the very top with A.analyzePattern(). Here the
      // actual numerical factorization is performed.
      solver.factorize(A);
      if (solver.info() != Eigen::Success) {
        // Decomposition failed. Maybe return kLinearFactorizationFailed.
        return FBSolverResult::kLinearSolverFailed;
      }
      dgamma = solver.solve(Fg);
      if (solver.info() != Eigen::Success) {
        // Solving failed.
        return FBSolverResult::kLinearSolverFailed;
      }

      // Update dv:
      GcT.Multiply(dgamma, &xv_aux);     // xv_aux = GcT * dgamma
      get_Minv().Multiply(xv_aux, &dv);  // dv = Mi * GcT * dgamma
      dv -= Fv;                          // dv = -Fv + Mi * GcT * dgamma
      //dv = -Fv + Mi_times_GT * dgamma;

      ExtractNormal(dgamma, &dpi);
      ExtractTangent(dgamma, &dbeta);

      // Store v += dv and g += dg in state_kp.
      state_kp = state_;
      state_kp.mutable_v() += dv;
      state_kp.mutable_gamma() += dgamma;
      state_kp.mutable_lambda() += state_.cache().dlambda;

      // Limit the final update either by LS or so that we dont cross outside
      // the feasible region.
      T alpha = LimitNormalUpdate(state_, state_kp, outer_iter);
      PRINT_VAR(alpha);

      // Perform the actual solver state update.
      state_.mutable_v() += alpha * dv;
      state_.mutable_gamma() += alpha * dgamma;
      state_.mutable_lambda() += alpha * state_.cache().dlambda;

      // Update constraints to new state_ so that we can compute residual.
      ValidateNormalConstraintsCache(state_);

      // Check inner loop convergence.
      double gpi_max_norm;
      const bool inner_converged = CheckInnerLoopConvergenceCriteria(
          state_.cache().gpi, state_.m(), &gpi_max_norm);
      PRINT_VAR(inner_converged);
      PRINT_VAR(gpi_max_norm);

      if (inner_converged) break;
    }  // inner_iter

    if( outer_iter == 0) {
      stats_.initialization_iters = num_inner;
    }

    // Update complementarity slackness parameter.
    if (state_.m() > m_final) {
      state_.mutable_m() *= parameters_.delta;
    }

    //if (outer_iter >= 1) {      
      const auto& vc = EvalVc(state_);
      const auto& vc_km = EvalVc(state_km);

      dvc = vc - vc_km;
      double dvc_max_norm;
      bool dvc_converged =
          CheckOuterLoopConvergenceCriteria(vc, dvc, &dvc_max_norm);
  
      // TODO: consider also checking the norm of scaling_factor.*pi (same
      // units)
      PRINT_VAR(dvc_converged);
      PRINT_VAR(dvc_max_norm);

      stats_.Update(dvc_max_norm, 0.0 /*update with true value of phi */,
                    num_inner);
      if (dvc_converged) {
        return FBSolverResult::kSuccess;
      }
    //}    

    // Store previous state so that we can compute the outer loop's convergence
    // error.
    state_km = state_;
  }  // outer_iter

  return FBSolverResult::kMaxIterationsReached;
}

}  // namespace solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::solvers::FBSolver)
