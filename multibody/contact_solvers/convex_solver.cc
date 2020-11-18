#include "drake/multibody/contact_solvers/convex_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using Eigen::SparseMatrix;
using Eigen::SparseVector;

template <typename T>
ContactSolverStatus ConvexSolver<T>::SolveWithGuess(
    const T& time_step, const SystemDynamicsData<T>& dynamics_data,
    const PointContactData<T>& contact_data, const VectorX<T>& v_guess,
    ContactSolverResults<T>* results) {
  pre_proc_data_ = PreProcessData(time_step, dynamics_data, contact_data);
  state_.Resize(pre_proc_data_.nv, pre_proc_data_.nc);

  const int nv = dynamics_data.num_velocities();
  const int nc = contact_data.num_contacts();

  // Aliases to data.
  const auto& Ainv = dynamics_data.get_Ainv();
  const auto& v_star = dynamics_data.get_v_star();
  const auto& Jc = contact_data.get_Jc();
  const auto& mu = contact_data.get_mu();

  // Aliases to pre-processed (const) data.
  const auto& W = pre_proc_data_.W;
  const auto& N = pre_proc_data_.N;
  const auto& r = pre_proc_data_.r;
  const auto& vc_stab = pre_proc_data_.vc_stab;
  const auto& vc_star = pre_proc_data_.vc_star;
  const auto& Rt = pre_proc_data_.Rt;
  const auto& Rn = pre_proc_data_.Rn;
  const auto& R = pre_proc_data_.R;
  const auto& Rinv = pre_proc_data_.Rinv;
  const auto& mu_tilde = pre_proc_data_.mu_tilde;

  // Aliases to solver's state.
  auto& gamma = state_.mutable_gamma();
  auto& nu = state_.mutable_cache().nu;
  auto& vc = state_.mutable_cache().vc;

  // Aliases to parameters.
  const int max_iters = parameters_.max_iterations;
  //const double omega = parameters_.relaxation;

  stats_ = {};
  if (nc == 0) {
    results->Resize(nv, nc);
    results->tau_contact.setZero();
    results->v_next = dynamics_data.get_v_star();
    stats_.iterations++;
    stats_.iteration_errors.push_back(ConvexErrorMetrics{});
    stats_history_.push_back(stats_);
    return ContactSolverStatus::kSuccess;
  }

  // Set initial guess.
  // Given v_guess, we can compute the analytical Inverse Dynamics to initialize
  // the solver, γ⁰ = γ(v⁰).  
  Jc.Multiply(v_guess, &vc);
  VectorX<T> y = -(Rinv.asDiagonal() * (vc - vc_stab));
  ProjectAllImpulses(pre_proc_data_, y, &gamma);  
  // gamma.setZero();  // If we don't know any better.

  // Below we use index k to denote the iteration. Hereinafter we'll adopt the
  // convention of appending a trailing _kp ("k plus") to refer to the next
  // iterate k+1.
  State state_kp(state_);  // Next iteration, k+1, state.
  // Aliases into state_kp.
  VectorX<T>& gamma_kp = state_kp.mutable_gamma();
  VectorX<T>& nu_kp = state_kp.mutable_cache().nu;
  VectorX<T>& vc_kp = state_kp.mutable_cache().vc;
  VectorX<T>& gamma_id_kp = state_kp.mutable_cache().gamma_id;

  // Reset stats.
  stats_.num_contacts = num_contacts();

  // Factorize N.
  Eigen::SparseLU<SparseMatrix<T>> Ninv;
  Ninv.analyzePattern(N);
  Ninv.factorize(N);
  if (Ninv.info() != Eigen::Success) {
    throw std::runtime_error("Factorization failed.");
    return ContactSolverStatus::kFailure;
  }

  SparseMatrix<T> X;  // X = W*Pi + R.
  X.resize(3 * nc, 3 * nc);
  X = N;  // it has the same pattern.
  Eigen::SparseLU<SparseMatrix<T>> Xinv;
  Xinv.analyzePattern(X);

  // Start iteration.
  for (int k = 0; k < max_iters; ++k) {
    //const VectorX<T> rhs = -(r + W * (gamma - y));

    // Compute block diagonal matrix Pi.
    std::vector<Matrix3<T>> Pi_blocks(nc);
    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      const auto y_ic = y.template segment<3>(ic3);
      Pi_blocks[ic] =
          CalcProjectionGradient(mu(ic), mu_tilde(ic), Rt(ic) / Rn(ic), y_ic);
      // CalcProjectionMatrix(mu(ic), mu_tilde(ic), Rt(ic) / Rn(ic), y_ic);
    }
    SparseMatrix<T> Pi(3 * nc, 3 * nc);
    FillSparseBlockDiagonalMatrix(Pi_blocks, &Pi);

    X = R.asDiagonal();
    X = X + W * Pi;

    Xinv.factorize(X);
    if (Xinv.info() != Eigen::Success) {
      throw std::runtime_error("Factorization of X failed.");
      return ContactSolverStatus::kFailure;
    }

    const VectorX<T> F = W * gamma + R.asDiagonal() * y + r;
    const VectorX<T> rhs = -F;
    const VectorX<T> dy = Xinv.solve(rhs);

    T alpha = parameters_.relaxation;
    if (parameters_.max_ls_iters > 0) {
      // Perform line search on ||F||.
      const T t = parameters_.ls_factor;
      VectorX<T> y_alpha = y + alpha * dy;
      VectorX<T> gamma_alpha(3 * nc);
      ProjectAllImpulses(pre_proc_data_, y_alpha, &gamma_alpha);
      VectorX<T> Falpha = W * gamma_alpha + R.asDiagonal() * y_alpha + r;
      T Fnorm_prev = Falpha.norm();
      for (int ils = 0; ils < parameters_.max_ls_iters; ++ils) {
        stats_.ls_iterations++;
        alpha *= t;
        y_alpha = y + alpha * dy;
        ProjectAllImpulses(pre_proc_data_, y_alpha, &gamma_alpha);
        Falpha = W * gamma_alpha + R.asDiagonal() * y_alpha + r;
        T Fnorm = Falpha.norm();
        if (Fnorm_prev < Fnorm) {
          alpha /= t;  // Back to previous alpha that was better.
          break;
        }
        Fnorm_prev = Fnorm;
      }
    }
    const VectorX<T> y_kp = y + alpha * dy;
    //const VectorX<T> y_kp = y + parameters_.relaxation * dy;

    //const VectorX<T> gamma_star = gamma + y_kp - y;
    //ProjectAllImpulses(pre_proc_data_, gamma_star, &gamma_kp);
    ProjectAllImpulses(pre_proc_data_, y_kp, &gamma_kp);

    // N.B. While the solution of the optimization problem is only formulated on
    // impulses, here we update velocities with the only purpose of computing a
    // number of other convergence metrics that do require velocities.
    // Update cached quantities.
    // Also compute cost.
    const VectorX<T> W_x_gamma = W * gamma_kp;
    const VectorX<T> N_x_gamma = W_x_gamma + R.asDiagonal() * gamma_kp;
    const T ell_kp = 0.5 * gamma_kp.dot(N_x_gamma) + gamma_kp.dot(r);
    vc_kp = W_x_gamma + vc_star;

    // Compute analytical inverse dynamics to obtain an error per iteration.
    // N.B. This computation can be avoided if we don't want ot monitor this
    // error.
    CalcInverseDynamics(pre_proc_data_, vc_kp, &gamma_id_kp);

    // Verify convergence and update stats.
    ConvexErrorMetrics error_metrics;
    error_metrics.ell = ell_kp;
    const bool converged = VerifyConvergenceCriteria(
        vc, vc_kp, gamma, gamma_kp, gamma_id_kp, &error_metrics.vc_err,
        &error_metrics.gamma_err, &error_metrics.id_err);

    // Compute the maximum effective stiction tolerance.
    using std::max;
    error_metrics.vs_max = -1.0;  // Value for all sliding contacts.
    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      const auto g_ic = gamma_kp.template segment<3>(ic3);
      const auto gt = g_ic.template head<2>();
      const T& gn = g_ic(2);

      const auto vc_ic = vc_kp.template segment<3>(ic3);
      const auto vt = vc_ic.template head<2>();

      const T gr = gt.norm();      
      const T vr = vt.norm();
      if (gr < mu(ic) * gn) {  // In stiction.
        const T vs = mu(ic) * gn / (gr + 1.0e-14) * vr;
        error_metrics.vs_max = max(vs, error_metrics.vs_max);
      }
    }

      stats_.iteration_errors.push_back(error_metrics);
    stats_.iterations++;

    // Update state for the next iteration.
    state_ = state_kp;
    y = y_kp;
    if (converged) break;
  }

  stats_history_.push_back(stats_);

  // N.B. With PGS we always return a solution even if we reach the maximum
  // number of iterations before convergence. We monitor errors across solutions
  // steps.

  results->Resize(nv, nc);
  VectorX<T>& tau_c = results->tau_contact;
  VectorX<T>& v = results->v_next;

  // Given the impulses, update generalized and contact velocities.
  // TODO(amcastro-tri): Move the update of momentum into ContactSolver.
  // Update generalized velocities; v = v* + M⁻¹⋅Jᵀ⋅γ.
  Jc.MultiplyByTranspose(gamma_kp, &tau_c);  // tau_c = Jᵀ⋅γ
  Ainv.Multiply(tau_c, &v);                  // v = M⁻¹⋅Jᵀ⋅γ
  v += v_star;                               // v = v* + M⁻¹⋅Jᵀ⋅γ
  Jc.Multiply(v, &vc);                       // vc = J⋅v

  // Pack results into ContactSolverResults.
  ExtractNormal(vc, &results->vn);
  ExtractTangent(vc, &results->vt);
  ExtractNormal(state_.gamma(), &results->fn);
  ExtractTangent(state_.gamma(), &results->ft);
  // N.B. While contact solver works with impulses, results are reported as
  // forces.
  results->fn /= time_step;
  results->ft /= time_step;
  results->tau_contact = tau_c / time_step;  

  return ContactSolverStatus::kSuccess;
}

template <typename T>
void ConvexSolver<T>::ProjectAllImpulses(const PreProcessedData& data,
                                         const VectorX<T>& y,
                                         VectorX<T>* gamma) const {
  const VectorX<T>& Rn = data.Rn;
  const VectorX<T>& Rt = data.Rt;
  const VectorX<T>& mu = data.mu;
  const VectorX<T>& mu_tilde = data.mu_tilde;

  for (int ic = 0, ic3 = 0; ic < num_contacts(); ic++, ic3 += 3) {
    auto y_ic = y.template segment<3>(ic3);
    const Vector3<T> gamma_ic =
        ProjectImpulse(mu(ic), mu_tilde(ic), Rt(ic) / Rn(ic), y_ic);
    gamma->template segment<3>(ic3) = gamma_ic;
  }
}

template <typename T>
typename ConvexSolver<T>::PreProcessedData
ConvexSolver<T>::PreProcessData(
    const T& time_step, const SystemDynamicsData<T>& dynamics_data,
    const PointContactData<T>& contact_data) const {
  const int nc = contact_data.num_contacts();
  const int nv = dynamics_data.num_velocities();
  PreProcessedData data;
  data.Resize(nv, nc);

  // Aliases to data.
  const auto& Ainv = dynamics_data.get_Ainv();
  const auto& v_star = dynamics_data.get_v_star();
  const auto& Jc = contact_data.get_Jc();

  if (nc != 0) {
    data.mu = contact_data.get_mu();  // just a handy copy.

    Jc.Multiply(v_star, &data.vc_star);    

    auto& W = data.W;
    this->FormDelassusOperatorMatrix(Jc, Ainv, Jc, &W);
    // Compute scaling factors, one per contact.
    // Notice mi and gi contain the diagonal component R.
    auto& gi = data.gi;
    auto& mi = data.mi;
    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      // 3x3 diagonal block. It might be singular, but definitely non-zero.
      // That's why we use an rms norm.
      const auto& Wii = W.block(ic3, ic3, 3, 3);
      gi(ic) = Matrix3<T>(Wii).trace() / 3;
      mi(ic) = 1.0 / gi(ic);
    }

    // Estimate regularization parameters.
    // We estimate a diagonal regularization Rn as a factor of the gᵢ =
    // trace(Wii)/3.
    // That is, Rn = eps * gᵢ.
    // The value of eps is estimated so that the numerical compliance time scale
    // is the contact_period below.

    // We note that compliance frequency is ω₀² = k/mᵢ.
    // We propose a period T₀ = α⋅dt and frequency ω₀ = 2π/T₀.
    // Then the stiffness will be k = ω₀² mᵢ.
    // Since Rn = (dt²k)⁻¹ we have that: Rn = gᵢ/(ω₀dt)² = ε⋅gᵢ.
    // With ε = 1/(ω₀dt)².

    const T contact_period = parameters_.alpha * time_step;
    const T w0 = 2.0 * M_PI / contact_period;
    const T w0_hat = w0 * time_step;
    const T eps = 1.0 / (w0_hat * w0_hat);
    // For the tangential compliance we use Rt = Rn * Rt_factor.
    // Usually we look for Rt_factor << 1
    const auto& mu = data.mu;
    auto& Rt = data.Rt;
    auto& Rn = data.Rn;
    auto& R = data.R;
    auto& Rinv = data.Rinv;
    auto& mu_tilde = data.mu_tilde;
    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      Rn(ic) = eps * gi(ic);
      Rt(ic) = parameters_.Rt_factor * Rn(ic);
      R.template segment<3>(ic3) = Vector3<T>(Rt(ic), Rt(ic), Rn(ic));
      Rinv.template segment<3>(ic3) =
          Vector3<T>(Rt(ic), Rt(ic), Rn(ic)).cwiseInverse();
      mu_tilde(ic) = sqrt(Rt(ic) / Rn(ic)) * mu(ic);
    }

    // Add regularization to Delassus operator into N = W + diag(R).
    auto& N = data.N;
    N = R.asDiagonal();
    N += W;  // maybe N = W, then N += R.asDiagonal(), but it might be some
               // diagonal elements are zero?

    // Recompute diagonal preconditioning.
    //for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
//      gi(ic) += (2.0 * Rt(ic) + Rn(ic)) / 3.0;  // += trace(Ri).
  //    mi(ic) = 1.0 / gi(ic);
    //}

    // Stabilization velocity.
    // TODO: explore Todorov's stabilization velocity.
    const auto& phi0 = contact_data.get_phi0();
    auto& vc_stab = data.vc_stab;
    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      // For now we'll use Anitescu's term b.
      Vector3<T> b(0.0, 0.0, phi0(ic));
      vc_stab.template segment<3>(ic3) = -b;
    }
    data.r = data.vc_star - vc_stab;

    PRINT_VARn(W.nonZeros());
    PRINT_VARn(MatrixX<T>(W));
    PRINT_VARn(N.nonZeros());
    PRINT_VARn(MatrixX<T>(N));
    PRINT_VAR(R.transpose());
    PRINT_VAR(data.r.transpose());
    PRINT_VAR(data.vc_star.transpose());
    PRINT_VAR(data.vc_stab.transpose());
    PRINT_VAR(gi.transpose());
    PRINT_VAR(mi.transpose());
  }

  return data;
}

template <typename T>
bool ConvexSolver<T>::VerifyConvergenceCriteria(
    const VectorX<T>& vc, const VectorX<T>& vc_kp, const VectorX<T>& gamma,
    const VectorX<T>& gamma_kp, const VectorX<T>& gamma_id,
    double* vc_err, double* gamma_err, double* id_err) const {
  using std::max;  

  // Scale impulse vector to have units of velocity.
  auto scale_vector = [this](const VectorX<T>& g) {
    const auto& gi = pre_proc_data_.gi;
    VectorX<T> s(g.size());
    for (int ic = 0, ic3 = 0; ic < num_contacts(); ic++, ic3 += 3) {
      s.template segment<3>(ic3) = g.template segment<3>(ic3) * gi(ic);
    }
    return s;
  };

  auto is_converged = [this](const VectorX<T>& s, const VectorX<T>& ds,
                             T* max_norm) {
    using std::max;
    auto within_error_bounds = [&p = parameters_](const T& error,
                                                  const T& scale) {
      const T bounds = p.abs_tolerance + p.rel_tolerance * scale;
      return error < bounds;
    };

    *max_norm = 0;
    bool converged = true;
    for (int ic = 0, ic3 = 0; ic < num_contacts(); ic++, ic3 += 3) {
      const auto s_ic = s.template segment<3>(ic3);
      const auto ds_ic = ds.template segment<3>(ic3);
      const T s_norm = s_ic.norm();
      const T s_err = ds_ic.norm();
      *max_norm = max(*max_norm, s_err);
      if (!within_error_bounds(s_err, s_norm)) {
        converged = false;
      }
    }
    return converged;
  };

  // We scale impulses to have units of velocity, s = gi * gamma.
  const VectorX<T> s = scale_vector(gamma);
  const VectorX<T> s_kp = scale_vector(gamma_kp);
  const VectorX<T> s_id = scale_vector(gamma_id);

  VectorX<T> de = (vc_kp - vc);  
  bool converged = is_converged(vc, de, vc_err);

  de = (s_kp - s);
  // N.B. Notice we purposely placed && converged "after" is_converged() so
  // that gamma_err gests computed.
  converged = is_converged(s, de, gamma_err) && converged;

  // Comparison with analytical inverse dynamics.
  de = (s_kp - s_id);
  converged = is_converged(s, de, id_err) && converged;
  //*id_err = (s_kp - s_id).template lpNorm<Eigen::Infinity>();

  return converged;

#if 0
  for (int ic = 0, ic3 = 0; ic < num_contacts(); ic++, ic3 += 3) {
    auto within_error_bounds = [&p = parameters_](const T& error,
                                                  const T& scale) {
      const T bounds = p.abs_tolerance + p.rel_tolerance * scale;
      return error < bounds;
    };
    // Check velocity convergence.
    const auto vci = vc.template segment<3>(ic3);
    const auto vci_kp = vc_kp.template segment<3>(ic3);
    const T vc_norm = vci.norm();
    const T vci_err = (vci_kp - vci).norm() / omega;
    *vc_err = max(*vc_err, vci_err);
    if (!within_error_bounds(vci_err, vc_norm)) {
      converged = false;
    }

    // Check impulse convergence. Scaled to velocity so that its
    // convergence metric is compatible with that of contact velocity.
    const auto gic = gamma.template segment<3>(ic3);
    const auto gic_kp = gamma_kp.template segment<3>(ic3);
    const T g_norm = gic.norm() * gi(ic);
    T g_err = (gic_kp - gic).norm() * gi(ic) / omega;
    *gamma_err = max(*gamma_err, g_err);
    if (!within_error_bounds(g_err, g_norm)) {
      converged = false;
    }
        }
#endif
}

// Returns tuple with (gamma, that, P(that)).
template <typename T>
Matrix3<T> ConvexSolver<T>::CalcProjectionGradient(
    const T& mu, const T& mu_tilde, const T& Rt_over_Rn,
    const Eigen::Ref<const Vector3<T>>& y) const {
  using std::sqrt;
  // Notation:
  //   yn: y(2)
  //   yt: y.head<2>();
  //   yr: yt.norm()
  // We use "tilde" to denote the change of variables:
  //   y_tilde = sqrt(R) * y
  // with R = diag({Rt, Rt, Rn})
  //
  // mu_tilde = mu * sqrt(Rt/Rn)
  const T& yn = y(2);
  const Vector2<T> yt = y.template head<2>();
  const T yr_squared = yt.squaredNorm();
  const T yr = sqrt(yr_squared);

  // Region III: Inside the polar cone yn_tile < -mu_tilde * yr_tilde.
  // This is equivalent to: yn < -mu * Rt/Rn * yr
  // Notice that in the limit Rt/Rn --> 0 we recover the original Coulomb
  // cone.
  if (yn < -mu * Rt_over_Rn * yr) {
    return Matrix3<T>::Zero();
  }

  // Region I: Inside the friction cone.
  // yr_tilde <= mu_tilde * yn_tilde is equivalent to yr < mu * yn.
  if (yr <= mu * yn) {
    return Matrix3<T>::Identity();
  }

  // Region II: Inside the region between the cone and its polar. We need to
  // project.
  constexpr double kEpsilonSquared = 1.0e-14;
  const T yr_soft = sqrt(yr_squared + kEpsilonSquared);
  const Vector2<T> that = yt / yr_soft;
  const Matrix2<T> P = that * that.transpose();
  // We build G such that the projection is Π(γ) = G⋅γ.
  Matrix3<T> G;
  const T mu_tilde2 = mu_tilde * mu_tilde;  
  G.template topLeftCorner<2, 2>() = mu_tilde2 * P;
  G.template topRightCorner<2, 1>() = mu * that;
  G.template bottomLeftCorner<1, 2>() = mu * Rt_over_Rn * that.transpose();
  G(2, 2) = 1.0;
  G /= (1.0 + mu_tilde2);
  // N.B. In the limit Rt_over_Rn --> 0, G projects on the Coulomb cone:
  // G = |0 μt̂ |
  //     |0  1 |

  const T gn = (mu * Rt_over_Rn * yr_soft + yn) / (1.0 + mu_tilde2);

  // Add contribution that includes dthat/dyt.
  const Matrix2<T> Pperp = Matrix2<T>::Identity() - P;
  G.template topLeftCorner<2, 2>() += mu * gn / yr_soft * Pperp;

  // If we are here, then we are in Region II and we need to project:
  return G;
}

template <typename T>
Matrix3<T> ConvexSolver<T>::CalcProjectionMatrix(
    const T& mu, const T& mu_tilde, const T& Rt_over_Rn,
    const Eigen::Ref<const Vector3<T>>& gamma) const {
  using std::sqrt;
  // Notation:
  //   gn: gamma(2)
  //   gt: gamma.head<2>();
  //   gr: gt.norm()
  // We use "tilde" to denote the change of variables:
  //   gamma_tilde = sqrt(R) * gamma
  // with R = diag({Rt, Rt, Rn})
  //
  // mu_tilde = mu * sqrt(Rt/Rn)
  const T& gn = gamma(2);
  const Vector2<T> gt = gamma.template head<2>();
  const T gr_squared = gt.squaredNorm();
  const T gr = sqrt(gr_squared);

  // Region III: Inside the polar cone gn_tile < -mu_tilde * gr_tilde.
  // This is equivalent to: gn < -mu * Rt/Rn * gr
  // Notice that in the limit Rt/Rn --> 0 we recover the original Coulomb
  // cone.
  if (gn < -mu * Rt_over_Rn * gr) {
    return Matrix3<T>::Zero();
  }

  // Region I: Inside the friction cone.
  // gr_tilde <= mu_tilde * gn_tilde is equivalent to gr < mu * gn.
  if (gr <= mu * gn) {
    return Matrix3<T>::Identity();
  }

  // Region II: Inside the region between the cone and its polar. We need to
  // project.
  constexpr double kEpsilonSquared = 1.0e-14;
  const T gr_soft = sqrt(gr_squared + kEpsilonSquared);
  const Vector2<T> that = gt / gr_soft;
  // We build G such that the projection is Π(γ) = G⋅γ.
  Matrix3<T> G;
  const T mu_tilde2 = mu_tilde * mu_tilde;
  G.template topLeftCorner<2, 2>() = mu_tilde2 * that * that.transpose();
  G.template topRightCorner<2, 1>() = mu * that;
  G.template bottomLeftCorner<1, 2>() = mu * Rt_over_Rn * that.transpose();
  G(2, 2) = 1.0;
  G /= (1.0 + mu_tilde2);
  // N.B. In the limit Rt_over_Rn --> 0, G projects on the Coulomb cone:
  // G = |0 μt̂ |
  //     |0  1 |

  // If we are here, then we are in Region II and we need to project:
  return G;
}

template <typename T>
Vector3<T> ConvexSolver<T>::ProjectImpulse(
    const T& mu, const T& mu_tilde, const T& Rt_over_Rn,
    const Eigen::Ref<const Vector3<T>>& gamma) const {
  using std::sqrt;
  // Notation:
  //   gn: gamma(2)
  //   gt: gamma.head<2>();
  //   gr: gt.norm()
  // We use "tilde" to denote the change of variables:
  //   gamma_tilde = sqrt(R) * gamma
  // with R = diag({Rt, Rt, Rn})
  //
  // mu_tilde = mu * sqrt(Rt/Rn)
  const T& gn = gamma(2);
  const Vector2<T> gt = gamma.template head<2>();
  const T gr_squared = gt.squaredNorm();
  const T gr = sqrt(gr_squared);

  // Region III: Inside the polar cone gn_tile < -mu_tilde * gr_tilde.
  // This is equivalent to: gn < -mu * Rt/Rn * gr
  // Notice that in the limit Rt/Rn --> 0 we recover the original Coulomb
  // cone.
  if (gn < -mu * Rt_over_Rn * gr) {
    return Vector3<T>::Zero();
  }

  // Region I: Inside the friction cone.
  // gr_tilde <= mu_tilde * gn_tilde is equivalent to gr < mu * gn.
  if (gr <= mu * gn) {
    return gamma;
  }

  // Region II: Inside the region between the cone and its polar. We need to
  // project.
  constexpr double kEpsilonSquared = 1.0e-14;
  const T gr_soft = sqrt(gr_squared + kEpsilonSquared);
  const Vector2<T> that = gt / gr_soft;
  // We build G such that the projection is Π(γ) = G⋅γ.
  Matrix3<T> G;
  const T mu_tilde2 = mu_tilde * mu_tilde;
  G.template topLeftCorner<2, 2>() = mu_tilde2 * that * that.transpose();
  G.template topRightCorner<2, 1>() = mu * that;
  G.template bottomLeftCorner<1, 2>() = mu * Rt_over_Rn * that.transpose();
  G(2, 2) = 1.0;
  G /= (1.0 + mu_tilde2);
  // N.B. In the limit Rt_over_Rn --> 0, G projects on the Coulomb cone:
  // G = |0 μt̂ |
  //     |0  1 |

  // If we are here, then we are in Region II and we need to project:
  return G * gamma;
}

template <typename T>
void ConvexSolver<T>::CalcInverseDynamics(const PreProcessedData& data,
                                             const VectorX<T>& vc,
                                             VectorX<T>* gamma) const {
  const VectorX<T>& Rn = data.Rn;
  const VectorX<T>& Rt = data.Rt;
  const VectorX<T>& mu = data.mu;
  const VectorX<T>& mu_tilde = data.mu_tilde;
  const VectorX<T>& vc_stab = data.vc_stab;

  for (int ic = 0, ic3 = 0; ic < num_contacts(); ic++, ic3 += 3) {
    auto vc_stab_ic = vc_stab.template segment<3>(ic3);
    auto vc_ic = vc.template segment<3>(ic3);
    const Vector3<T> vc_tilde_ic = vc_ic - vc_stab_ic;

    // y = -R⁻¹⋅vc_tilde_ic
    const Vector3<T> y_ic(-vc_tilde_ic(0) / Rt(ic), -vc_tilde_ic(1) / Rt(ic),
                          -vc_tilde_ic(2) / Rn(ic));
    const Vector3<T> gamma_ic =
        ProjectImpulse(mu(ic), mu_tilde(ic), Rt(ic) / Rn(ic), y_ic);
    gamma->template segment<3>(ic3) = gamma_ic;
  }
}

template <typename T>
void ConvexSolver<T>::FillSparseBlockDiagonalMatrix(
    const std::vector<Matrix3<T>>& blocks, SparseMatrix<T>* S) const {
  const int nb = blocks.size();
  const int n = 3 * nb;
  S->resize(n, n);
  S->reserve(Eigen::VectorXi::Constant(n, 3));  // 3 non-zeros per column.
  for (int b = 0; b < nb; b++) {
    const Matrix3<T>& B = blocks[b];
    for (int i = 0; i < 3; ++i) {
      const int ik = 3 * b + i;
      for (int j = 0; j < 3; ++j) {
        const int jk = 3 * b + j;
        S->insert(ik, jk) = B(i, j);
      }
    }
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::ConvexSolver<
    double>;
