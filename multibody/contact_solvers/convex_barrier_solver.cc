#include "drake/multibody/contact_solvers/convex_barrier_solver.h"

#include "fmt/format.h"

#include <numeric>
#include <fstream>
#include <string>

#include<iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using Eigen::SparseMatrix;
using Eigen::SparseVector;

template <typename T>
ContactSolverStatus ConvexBarrierSolver<T>::SolveWithGuess(
    const T& time_step, const SystemDynamicsData<T>& dynamics_data,
    const PointContactData<T>& contact_data, const VectorX<T>& v_guess,
    ContactSolverResults<T>* results) {
  using std::abs;
  using std::max;
  pre_proc_data_ = PreProcessData(time_step, dynamics_data, contact_data);
  state_.Resize(pre_proc_data_.nv, pre_proc_data_.nc);

  const int nv = dynamics_data.num_velocities();
  const int nc = contact_data.num_contacts();
  const int nc3 = 3 * nc;

  // Aliases to data.
  const auto& Ainv = dynamics_data.get_Ainv();
  const auto& v_star = dynamics_data.get_v_star();
  const auto& Jc = contact_data.get_Jc();
  const auto& mu = contact_data.get_mu();

  // Aliases to pre-processed (const) data.
  const auto& N = pre_proc_data_.N;
  const auto& W = pre_proc_data_.W;
  const auto& r = pre_proc_data_.r;
  const auto& vc_stab = pre_proc_data_.vc_stab;
  const auto& vc_star = pre_proc_data_.vc_star;
  const auto& mi = pre_proc_data_.mi;
  //const auto& Rt = pre_proc_data_.Rt;
  //const auto& Rn = pre_proc_data_.Rn;
  const auto& R = pre_proc_data_.R;
  //const auto& mu_tilde = pre_proc_data_.mu_tilde;

  // Aliases to solver's state.
  VectorX<T>& gamma = state_.mutable_gamma();
  auto& kappa = state_.mutable_kappa();
  VectorX<T>& vc = state_.mutable_cache().vc;

  // Aliases to parameters.
  const int max_iters = parameters_.max_iterations;

  stats_ = {};
  if (nc == 0) {
    results->Resize(nv, nc);
    results->tau_contact.setZero();
    results->v_next = dynamics_data.get_v_star();
    stats_.iterations++;
    stats_.iteration_errors.push_back(ConvexBarrierSolverErrorMetrics{});
    stats_history_.push_back(stats_);
    return ContactSolverStatus::kSuccess;
  }

  // Set initial guess.
  // Given v_guess, we can compute the analytical Inverse Dynamics to initialize
  // the solver, γ⁰ = γ(v⁰).  
  Jc.Multiply(v_guess, &vc);

  CalcInitialCondition(vc, &gamma);
  PRINT_VAR(gamma.transpose());

  // Multiplier nu.
  const VectorX<T> nu = -(vc - vc_stab + R.asDiagonal() * gamma);
  PRINT_VAR(nu.transpose());
  const T kappa_mean = -nu.dot(gamma) / nc;
  PRINT_VAR(kappa_mean);
  kappa = VectorX<T>::Constant(nc, abs(kappa_mean));

  // Compute ID solution for comparison.
  VectorX<T> gamma_kappa(nc3);
  VectorX<T> gamma_id(nc3);
  CalcInverseDynamics(pre_proc_data_, kappa, vc, &gamma_id, &gamma_kappa);
  PRINT_VAR(gamma_id.transpose());
  PRINT_VAR(gamma_kappa.transpose());

#if 0
  // N.B. At this point kappa has trash. However it is not used to compute
  // gamma_kappa. We only compute gamma_id, not gamma_kappa.
  VectorX<T> gamma_id(3 * nc);
  CalcInverseDynamics(pre_proc_data_, kappa, vc, &gamma_id);
  const T epsilon = 1.0;  // = sin(Δθ)
  // Estimate the norm of gamma_ic and vc_ic as the rms of the full vectors.
  const T vc_ic_mean_norm = max(vc_star.norm(), vc.norm()) / sqrt(1.0 * nc);
  PRINT_VAR(vc_ic_mean_norm);

  kappa = epsilon * vc_ic_mean_norm * vc_ic_mean_norm * mi;

  //const T gamma_ic_mean_norm = gamma_id.norm() / sqrt(1.0 * nc);
  //PRINT_VAR(gamma_ic_mean_norm);  
  //const T kappa_mean = epsilon * gamma_ic_mean_norm * vc_ic_mean_norm;
  //kappa = VectorX<T>::Constant(nc, kappa_mean);
  PRINT_VAR(kappa.transpose());

  //EstimateKappaGivenAccuracy(true, 1.0, gamma_id, vc, &kappa);
  // Safety factor.
  //kappa *= 1000.0;
  //PRINT_VAR(kappa.transpose());

  //vc = W * gamma_id + vc_star;
  //EstimateKappa(gamma_id, vc, &kappa);
  //PRINT_VAR(kappa.transpose());

  // Now that we estimated kappa with gamma_ID, we'll compute gamma_kappa as the
  // initial guess (since gamma_ID might lead to s = 0).
  //VectorX<T> gamma_kappa(3 * nc);  // = gamma_ID, discarded.
  //Jc.Multiply(v_guess, &vc);
  CalcInverseDynamics(pre_proc_data_, kappa, vc, &gamma_id, &gamma);

  PRINT_VAR(gamma_id.transpose());
  PRINT_VAR(gamma.transpose());
#endif

  // Below we use index k to denote the iteration. Hereinafter we'll adopt the
  // convention of appending a trailing _kp ("k plus") to refer to the next
  // iterate k+1.
  State state_kp(state_);  // Next iteration, k+1, state.
  // Aliases into state_kp.
  VectorX<T>& gamma_kp = state_kp.mutable_gamma();
  auto& vc_kp = state_kp.mutable_cache().vc;

  // Reset stats.
  stats_.num_contacts = num_contacts();

  // Hessian of the unconstrained cost.
  SparseMatrix<T> hessian_ell;  // H = N + ∇²B  
  // The Hessian as the same structure of N. At pre-proc, we already added
  // entries (zero if needed) to the 3x3 block diagonal elements of N.
  hessian_ell.resize(3 * nc, 3 * nc);
  hessian_ell = N;

  // TODO: replace with Cholesky factorization since H is SPD.
  Eigen::SparseLU<SparseMatrix<T>> hessian_inv;
  hessian_inv.analyzePattern(hessian_ell);

  // Start iteration.
  for (int k = 0; k < max_iters; ++k) {
    std::cout << std::string(80,'=') << std::endl;
    std::cout << std::string(80,'=') << std::endl;
    PRINT_VAR(k);
    std::cout << std::endl;

    VectorX<T> ell_gradient(nc3);
    VectorX<T> barrier_gradient(nc3);
    CalcUnconstrainedCostAndGradients(state_, &ell_gradient, &hessian_ell,
                                      &barrier_gradient);    
    const T cos_nu_gamma =
        gamma.dot(barrier_gradient) / gamma.norm() / barrier_gradient.norm();
    PRINT_VAR(cos_nu_gamma);

    //PRINT_VARn(MatrixX<T>(hessian_ell));
    //PRINT_VAR(MatrixX<T>(hessian_ell).eigenvalues().transpose());
    //PRINT_VAR(hessian_ell.norm());

    hessian_inv.factorize(hessian_ell);
    PRINT_VAR(hessian_inv.determinant());
    if (hessian_inv.info() != Eigen::Success) {
      throw std::runtime_error("Factorization of hessian failed.");
      return ContactSolverStatus::kFailure;
    }

    // N.B. gamma + dgamma might be outside the feasible set. However,
    // we do know that ell_b decreases in this direction.
    VectorX<T> dgamma = hessian_inv.solve(-ell_gradient);

    const T cos_grad_dgamma =
        ell_gradient.dot(dgamma) / ell_gradient.norm() / dgamma.norm();
    PRINT_VAR(ell_gradient.norm());
    PRINT_VAR(dgamma.norm());
    PRINT_VAR(cos_grad_dgamma);

    //const VectorX<T> gamma_plus = gamma + dgamma;
    //const VectorX<T> vc_plus = W * gamma_plus + vc_star;
    const VectorX<T> dummy;
   // = -(vc_plus - vc_stab).cwiseQuotient(R);
    T beta = LimitToFeasibleRegion(gamma, dgamma, dummy, kappa);    
    PRINT_VAR(beta);
    beta *= parameters_.sigma;

    dgamma = beta * dgamma;



    const T alpha = DoNewtonLineSearch(state_, dgamma, &stats_, &hessian_ell);
    PRINT_VAR(alpha);

    gamma_kp = gamma + alpha * dgamma;

    PRINT_VAR(gamma_kp.transpose());

    // TODO: Update kappa dynamically.

    bool converged = false;
    {  // Stats
      // N.B. While the solution of the optimization problem is only formulated
      // on impulses, here we update velocities with the only purpose of
      // computing a number of other convergence metrics that do require
      // velocities. Update cached quantities. Also compute cost.
      const VectorX<T> W_x_gamma = W * gamma_kp;
      const VectorX<T> N_x_gamma = W_x_gamma + R.asDiagonal() * gamma_kp;
      const T ell_kp = 0.5 * gamma_kp.dot(N_x_gamma) + gamma_kp.dot(r);
      vc_kp = W_x_gamma + vc_star;

      // Compute analytical inverse dynamics to obtain an error per iteration.
      // N.B. This computation can be avoided if we don't want ot monitor this
      // error.
      //VectorX<T>& gamma_id = state_kp.mutable_cache().gamma_id;
      //VectorX<T> gamma_kappa(3 * nc);
      CalcInverseDynamics(pre_proc_data_, kappa, vc_kp, &gamma_id,
                          &gamma_kappa);

      PRINT_VAR(gamma_id.transpose());                          
      PRINT_VAR(gamma_kappa.transpose());

      // Re-estimate kappa.
      if (parameters_.dynamic_kappa) {
        if (beta < 0.5) {
          state_kp.mutable_kappa() *= 10.0;
        } else {
          // N.B. Since this is an interior point method, gamma_kp has the
          // property of always having non-zero norm (in contrast to gamma_id).
          // Therefore we use gamma_kp to come up with an acceptable error.
          EstimateKappaGivenAccuracy(false, beta, gamma_kp, vc_kp,
                                     &state_kp.mutable_kappa());
        }
      }
      PRINT_VAR(state_kp.kappa().transpose());

      // Verify convergence and update stats.
      ConvexBarrierSolverErrorMetrics error_metrics;
      error_metrics.ell = ell_kp;
      error_metrics.beta = beta;
      error_metrics.id_rel_error =
          (gamma_id - gamma_kappa).norm() / gamma_id.norm();
      converged = VerifyConvergenceCriteria(
          vc, vc_kp, gamma, gamma_kp, gamma_kappa, &error_metrics.vc_err,
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
    }  // Stats

    // Update state for the next iteration.
    state_ = state_kp;
    
    if (converged) break;
  }

  stats_history_.push_back(stats_);

  results->Resize(nv, nc);
  VectorX<T>& tau_c = results->tau_contact;
  VectorX<T>& v = results->v_next;

  // Given the impulses, update generalized and contact velocities.
  // TODO(amcastro-tri): Move the update of momentum into ContactSolver.
  // Update generalized velocities; v = v* + M⁻¹⋅Jᵀ⋅γ.
  Jc.MultiplyByTranspose(gamma, &tau_c);  // tau_c = Jᵀ⋅γ
  Ainv.Multiply(tau_c, &v);                  // v = M⁻¹⋅Jᵀ⋅γ
  v += v_star;                               // v = v* + M⁻¹⋅Jᵀ⋅γ
  Jc.Multiply(v, &vc);                       // vc = J⋅v

  // Pack results into ContactSolverResults.
  ExtractNormal(vc, &results->vn);
  ExtractTangent(vc, &results->vt);
  ExtractNormal(gamma, &results->fn);
  ExtractTangent(gamma, &results->ft);
  // N.B. While contact solver works with impulses, results are reported as
  // forces.
  results->fn /= time_step;
  results->ft /= time_step;
  results->tau_contact = tau_c / time_step;  

  return ContactSolverStatus::kSuccess;
}

template <typename T>
void ConvexBarrierSolver<T>::ProjectAllImpulses(const PreProcessedData& data,
                                         const VectorX<T>& y,
                                         VectorX<T>* gamma) const {
  const VectorX<T>& mu = data.mu;
  const VectorX<T>& mu_tilde = data.mu_tilde;
  const VectorX<T>& Rt = data.Rt;
  const VectorX<T>& Rn = data.Rn;

  for (int ic = 0, ic3 = 0; ic < num_contacts(); ic++, ic3 += 3) {
    const auto y_ic = y.template segment<3>(ic3);
    const Vector3<T> gamma_ic = ProjectImpulse(mu(ic), mu_tilde(ic), Rt(ic) / Rn(ic), y_ic);
    gamma->template segment<3>(ic3) = gamma_ic;
  }
}

template <typename T>
typename ConvexBarrierSolver<T>::PreProcessedData
ConvexBarrierSolver<T>::PreProcessData(
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
    // N.B. We add full 3x3 diagonal blocks so that the pattern of N matches the
    // pattern of the Hessian even if those blocks are missing in N.
    std::vector<Matrix3<T>> diagonal_blocks(nc, Matrix3<T>::Zero());
    for (int ic = 0; ic < nc; ic++) {
      diagonal_blocks[ic] = Vector3<T>(Rt(ic), Rt(ic), Rn(ic)).asDiagonal();
    }
    InsertSparseBlockDiagonal(diagonal_blocks, &N);
    //N = R.asDiagonal();
    N += W;
    N.makeCompressed();

    // Recompute diagonal preconditioning.
    //for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    //  gi(ic) += (2.0 * Rt(ic) + Rn(ic)) / 3.0;  // += trace(Ri).
    //  mi(ic) = 1.0 / gi(ic);
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
bool ConvexBarrierSolver<T>::VerifyConvergenceCriteria(
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
  is_converged(s, de, id_err);
  // TODO: include this when the ID solution is on the unconstrained problem.
  //converged = is_converged(s, de, id_err) && converged;  

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
Matrix3<T> ConvexBarrierSolver<T>::CalcProjectionGradient(
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
  //constexpr double kEpsilonSquared = 1.0e-14;
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
Matrix3<T> ConvexBarrierSolver<T>::CalcProjectionMatrix(
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
  //constexpr double kEpsilonSquared = 1.0e-14;
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
Vector3<T> ConvexBarrierSolver<T>::ProjectImpulse(
    const T& mu, const T& mu_tilde, const T& Rt_over_Rn,
    const Eigen::Ref<const Vector3<T>>& gamma, Vector3<T>* fperp) const {
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

  // Build the projection direction fperp.
  //constexpr double kEpsilonSquared = 1.0e-14;
  const T gr_soft = sqrt(gr_squared + kEpsilonSquared);
  const Vector2<T> that = gt / gr_soft;
  fperp->template head<2>() = that;
  (*fperp)(2) = -mu * Rt_over_Rn;

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
void ConvexBarrierSolver<T>::CalcInverseDynamics(
    const PreProcessedData& data, const VectorX<T>& kappa, const VectorX<T>& vc,
    VectorX<T>* gamma, VectorX<T>* gamma_kappa) const {
  using std::abs;
  using std::sqrt;
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
    Vector3<T> fperp;  // The direction of the projection.
    const Vector3<T> gamma_id =
        ProjectImpulse(mu(ic), mu_tilde(ic), Rt(ic) / Rn(ic), y_ic, &fperp);
    gamma->template segment<3>(ic3) = gamma_id;
    if (gamma_kappa) {
      const T yr = y_ic.template head<2>().norm();
      const T yn = y_ic(2);
      if (yn < -mu(ic) * Rt(ic) / Rn(ic) * yr) {  // Inside polar cone.
        // The solution here lies on the revolute axis of the cone. The
        // derivative is discontinuous here and thus we use the generalized
        // gradient (or a smoothed cone if you will). Then what happens is that
        // the normal direction determines the value of gn and the tangential
        // directions accommodates to how offset y is.
        const T gn = 0.5 * (yn + sqrt(yn * yn + 4.0 * kappa(ic) / Rt(ic)));
        gamma_kappa->template segment<3>(ic3) = Vector3<T>(0.0, 0.0, gn);
      } else {
        const T delta = -CalcConeSdf(mu(ic), y_ic);
        const T factor = sqrt(kappa(ic) / Rt(ic));
        const T x = 0.5 * delta / factor;
        const T t = factor * (sqrt(x * x + 1) - abs(x));
        gamma_kappa->template segment<3>(ic3) = gamma_id - t * fperp;
      }
    }
  }
}

template <typename T>
void ConvexBarrierSolver<T>::InsertSparseBlockDiagonal(
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

template <typename T>
void ConvexBarrierSolver<T>::AddSparseBlockDiagonal(
    const std::vector<Matrix3<T>>& blocks, SparseMatrix<T>* S) const {
  const int nb = blocks.size();
  const int n = 3 * nb;
  DRAKE_DEMAND(S->rows() == n);
  DRAKE_DEMAND(S->cols() == n);
  //S->resize(n, n);
  //S->reserve(Eigen::VectorXi::Constant(n, 3));  // 3 non-zeros per column.
  for (int b = 0; b < nb; b++) {
    const Matrix3<T>& B = blocks[b];
    for (int i = 0; i < 3; ++i) {
      const int ik = 3 * b + i;
      for (int j = 0; j < 3; ++j) {
        const int jk = 3 * b + j;
        S->coeffRef(ik, jk) += B(i, j);
      }
    }
  }
}

template <typename T>
T ConvexBarrierSolver<T>::CalcUnconstrainedCostAndGradients(
    const State& state, VectorX<T>* ell_gradient,
    SparseMatrix<T>* ell_hessian, VectorX<T>* barrier_gradient) const {
  const int nc = pre_proc_data_.nc;
  DRAKE_DEMAND(ell_gradient != nullptr);
  DRAKE_DEMAND(ell_gradient->size() == 3 * nc);
  using std::log;
  // Pre-proc data.
  const auto& N = pre_proc_data_.N;
  const auto& r = pre_proc_data_.r;
  const auto& mu = pre_proc_data_.mu;

  // State.
  const VectorX<T>& gamma = state.gamma();
  const VectorX<T>& kappa = state.kappa();

  // Constrained cost & gradient.
  const VectorX<T> N_times_gamma = N * gamma;
  T ell = gamma.dot(0.5 * N_times_gamma + r);
  *ell_gradient = N_times_gamma + r;

  // Compute Gradient and Hessian for each barrier function.
  std::vector<Matrix3<T>> barrier_hessian_blocks(nc);  // = ∇²B
  for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    const auto gamma_ic = gamma.template segment<3>(ic3);
    Matrix3<T>* barrier_hessian_ic =
        (ell_hessian == nullptr) ? nullptr : &barrier_hessian_blocks[ic];
    Vector3<T> barrier_gradient_ic;
    const T barrier_ic = CalcBarrierAndGradients(
        kappa(ic), mu(ic), gamma_ic, &barrier_gradient_ic, barrier_hessian_ic);

    //if (barrier_hessian_ic){
    //  PRINT_VARn(*barrier_hessian_ic);
    //  PRINT_VAR(barrier_hessian_ic->eigenvalues().transpose());
    //}

    ell += barrier_ic;    
    ell_gradient->template segment<3>(ic3) += barrier_gradient_ic;

    if (barrier_gradient)
      barrier_gradient->template segment<3>(ic3) = barrier_gradient_ic;
  }
  if (ell_hessian != nullptr) {    
    // N.B. Here I assume that at pre-proc N was filled with at least zeros in
    // the block diagonal.
    *ell_hessian = N;  
    AddSparseBlockDiagonal(barrier_hessian_blocks, &*ell_hessian);
  }
  return ell;
}

// Assumption, γ ∈ F.
template <typename T>
T ConvexBarrierSolver<T>::CalcBarrierAndGradients(
    const T& kappa, const T& mu, const Eigen::Ref<const Vector3<T>>& y,
    Vector3<T>* gradient, Matrix3<T>* Hessian) const {
  DRAKE_DEMAND(gradient != nullptr);
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
  //constexpr double kEpsilon = 1.0e-7;
  //constexpr double kEpsilonSquared = kEpsilon * kEpsilon;
  const T yr_soft = sqrt(yr_squared + kEpsilonSquared);
  
  // N.B. Notice the "soft" definition of s(gamma) so that gradient and Hessian
  // are defined everywhere including gamma = 0.
  // N.B. Notice we add kEpsilon so that s > 0 enforces gn > 0.
  // N.B. This function IS convex in the neighborhood of epsilon around zero.
  const T s = mu * yn - yr_soft + kEpsilon;
  // Sanity check the assumption γ ∈ F.
  PRINT_VAR(s);
  DRAKE_DEMAND(s > 0.0);

  // Barrier.
  const T barrier = -kappa * log(s);

  // Gradient.
  const Vector2<T> that = yt / yr_soft;
  *gradient = Vector3<T>(that(0), that(1), -mu) * kappa / s;

  // Hessian.
  if (Hessian != nullptr) {
    const Matrix2<T> P = that * that.transpose();
    const Matrix2<T> Pperp = Matrix2<T>::Identity() - P;

    (*Hessian) << P, -mu * that, -mu * that.transpose(), mu * mu;
    (*Hessian) *= (kappa / s / s);

    Matrix2<T> HB2 = Pperp * kappa / s / yr_soft;
    Hessian->template topLeftCorner<2, 2>() += HB2;
  }

  return barrier;
}

template <typename T>
T ConvexBarrierSolver<T>::LimitToFeasibleRegion(
    const VectorX<T>& gamma, const VectorX<T>& dgamma,
    const VectorX<T>& y_plus,
    const VectorX<T>& kappa) const {
  using std::abs;
  using std::max;
  using std::min;
  using std::sqrt;
  const int nc = pre_proc_data_.nc;
  //const T kappa_min_factor = parameters_.kappa_min_factor;
  const auto& mu = pre_proc_data_.mu;
  //const auto& Rt = pre_proc_data_.Rt;
  T beta = 1.0;
  for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    const auto gamma_ic = gamma.template segment<3>(ic3);    
    const auto dgamma_ic = dgamma.template segment<3>(ic3);
    //const auto y_ic = y_plus.template segment<3>(ic3);

    const Vector3<T> gamma_plus = gamma_ic + dgamma_ic;
    const T s_plus = CalcConeSdf(mu(ic), gamma_plus);
    if (s_plus <= 0) {
      // If outside the cone, then we find intersection with it.
      const T beta_ic =
          LimitFeasibleRayWithinCone(mu(ic), gamma_ic, dgamma_ic, 0.0);
      PRINT_VAR(beta_ic);
      const T error = CalcConeSdf(mu(ic), gamma_ic + beta_ic * dgamma_ic);
      PRINT_VAR(error);
      //DRAKE_DEMAND(abs(error) < 1.0e-6);
      beta = min(beta, beta_ic);
    }
  }

#if 0
    const T sqrt_kappa = sqrt(kappa[ic]);

    const T s_tilde = sqrt(Rt(ic)) * s_plus;
    PRINT_VAR(s_tilde / sqrt_kappa);

    // Analytical estimation of s_kappa.
    const T delta = CalcConeSdf(mu(ic), y_ic);
    const T delta_tilde = -sqrt(Rt(ic)) * delta;
    const T x = 0.5 * delta_tilde / sqrt_kappa;
    const T s_kappa_tilde = sqrt_kappa * (sqrt(x * x + 1) - x);
    PRINT_VAR(s_kappa_tilde / sqrt_kappa);
    const T s_kappa = s_kappa_tilde / sqrt(Rt(ic));

    PRINT_VAR(delta_tilde);
    PRINT_VAR(x);

    // Estimation of s for 

    //const T kappa_min = kappa_min_factor * kappa[ic];
    //const T s_min = sqrt(kappa_min / Rt(ic));
    PRINT_VAR(sqrt(kappa[ic] / Rt(ic)));
    //const T s_min = kappa_min_factor * sqrt(kappa[ic] / Rt(ic));
    const T s_min = kappa_min_factor * s_kappa;

    // No need to limit if ray fully contained within s(gamma) > s_min.
    if (s_plus < s_min) {
      const T beta_ic =
          LimitFeasibleRayWithinCone(mu(ic), gamma_ic, dgamma_ic, s_min);
      PRINT_VAR(ic);
      PRINT_VAR(beta_ic);
      beta = min(beta, beta_ic);
    }
  }
#endif
  return beta;
}

template <typename T>
T ConvexBarrierSolver<T>::LimitFeasibleRayWithinCone(
    const T& mu, const Eigen::Ref<const Vector3<T>>& g,
    const Eigen::Ref<const Vector3<T>>& dg, const T& s_min) const {
  using std::max;
  using std::sqrt;

  // This method assumes.
  //  1. s(g) >> s_min
  //  2. s(g+dg) < s_min
  //  Therefore this leads to the case with two roots, a positive and a
  //  negative one.

  // We know that s(g) > 0, therefore ||g|| > 0 and we can use its norm to
  // normalize all coefficients in the resulting quadrating to avoid round-off
  // errors.
  const T g_scale = g.norm();

  // Sanity check assumptions.
  const T s_g = CalcConeSdf(mu, g);
  const T s_gplus = CalcConeSdf(mu, g + dg);

  PRINT_VAR(s_g / g_scale);
  PRINT_VAR(s_gplus / g_scale);
  PRINT_VAR(s_min / g_scale);

  DRAKE_DEMAND(s_g > s_min);  // g is inside cone s = s_min.
  DRAKE_DEMAND(s_gplus < s_min);  // gplus is outside cone s = s_min.

  const T kEpsilonScaled = kEpsilon / g_scale;
  const T kEpsilonScaledSquared = kEpsilonScaled * kEpsilonScaled;

  // Handy short names, scaled by g_scale.
  const T n = mu * g(2) / g_scale;
  const T dn = mu * dg(2) / g_scale;
  const Vector2<T> t = g.template head<2>() / g_scale;
  const Vector2<T> dt = dg.template head<2>() / g_scale;
  const T s = s_min - kEpsilonScaled;

  // Compute quadratic coefficients, they are all scaled with g_scale.
  const T a = dn * dn - dt.squaredNorm();
  const T b = 2.0 * (n * dn - s * dn - t.dot(dt));
  const T c =
      n * n + s * s - 2.0 * s * n - t.squaredNorm() - kEpsilonScaledSquared;

  PRINT_VAR(a);
  PRINT_VAR(b);
  PRINT_VAR(c);

  // We'll assume non-zero for now.
  DRAKE_DEMAND(abs(a) > std::numeric_limits<double>::epsilon());

  // Cases:
  //  1. a != 0 ==> two roots.
  //  2. a = 0, b != 0 ==> linear, one root. Ray tangent to s.
  //  3. a = 0, b = 0, c != 0 ==> no intersection.

  const T Delta = b * b - 4 * a * c;
  PRINT_VAR(Delta);
  DRAKE_DEMAND(Delta > 0);
  const T sqrt_Delta = sqrt(Delta);

  // To avoid loss of significance, when 4ac is relatively small compared
  // to b² (i.e. the square root of the discriminant is close to b), we use
  // Vieta's formula (α₁α₂ = c / a) to compute the second root given we
  // computed the first root without precision lost. This guarantees the
  // numerical stability of the method.
  const T numerator = -0.5 * (b + (b > 0.0 ? sqrt_Delta : -sqrt_Delta));
  const T b1 = numerator / a;
  const T b2 = c / numerator;

  // const T b1 = 0.5 * (-b + sqrt_Delta) / a;
  // const T b2 = 0.5 * (-b - sqrt_Delta) / a;

  //  auto soft_norm = [kEpsilonSquared](const Vector2<T>& v) {
  //  return sqrt(v.squaredNorm() + kEpsilonSquared);
  //};

  PRINT_VAR(b1);
  PRINT_VAR(b2);

  // const Vector3<T> g1 = g + b1 * dg;
  // const Vector3<T> g2 = g + b2 * dg;
  // PRINT_VAR(mu * g1(2) - s_min);
  // PRINT_VAR(soft_norm(g1.template head<2>()));
  // PRINT_VAR(mu *g2(2) - s_min);
  // PRINT_VAR(soft_norm(g2.template head<2>()));

  if (b1 > 0 && b2 > 0) {
    // We get two positive solutions actually because we squared the original
    // equation.
    // The original equation is
    //   μγₙ − s + ε = ‖γₜ‖ₛ > 0
    // however we solved for (μγₙ − s + ε)² = ‖γₜ‖ₛ².
    // Therefore the one valid solution will satisfy μγₙ − s > 0.
#if 0    
    const T gn1 = g(2) + b1 * dg(2);
    const T gt_norm1 = mu * gn1 - s_min;
    const T gn2 = g(2) + b2 * dg(2);
    const T gt_norm2 = mu * gn2 - s_min;
    DRAKE_DEMAND(gt_norm1 * gt_norm2 < 0);    
    if (gt_norm1 > 0)
      return b1;
    else
      return b2;
#endif
    // Simply choose the positive normal.
    const T beta = (g(2) + b1 * dg(2) >= 0) ? b1 : b2;
    PRINT_VAR(beta);
    DRAKE_DEMAND(beta <= 1.0);
    return beta;
  }

  // If we are here, there should at least be a single positive root.
  // Sanity check our assumption of two distinctint real roots.
  PRINT_VAR(b1 * b2);  
  DRAKE_DEMAND(b1 * b2 < 0.0);

  const T beta = max(b1, b2);
  PRINT_VAR(beta);

  DRAKE_DEMAND(beta <= 1.0);
  return beta;
}

template <typename T>
T ConvexBarrierSolver<T>::DoLineSearch(const State& state,
                                       const VectorX<T>& dgamma, ConvexBarrierSolverStats* stats) const {
  const int nc = pre_proc_data_.nc;
  const int nc3 = 3 * nc;

  // N.B. I could actually root search dell/dalpha = 0 with a Newton iteration.
  // TODO: try a Newton strategy.

  State state_alpha(state); 
  VectorX<T> ell_gradient(nc3);

  const auto& gamma = state.gamma();
  auto& gamma_alpha = state_alpha.mutable_gamma();

  // ell_prev = ell(alpha = 0).  
  T ell_prev = CalcUnconstrainedCostAndGradients(state, &ell_gradient);
  const T gradient_dalpha = ell_gradient.dot(dgamma);
  PRINT_VAR(gradient_dalpha);
  // The unconstrained cost should decrease in the direction of dgamma.
  DRAKE_DEMAND(gradient_dalpha);  
  T alpha = 1.0;
  for (int ils = 0; ils < parameters_.max_ls_iters; ++ils) {
    stats->ls_iterations++;
    alpha = parameters_.ls_factor * alpha;
    gamma_alpha = gamma + alpha * dgamma;
    const T ell = CalcUnconstrainedCostAndGradients(state_alpha, &ell_gradient);
    if (ell > ell_prev) {
      // Previous alpha wast better.
      alpha /= parameters_.ls_factor;
      break;
    }
    ell_prev = ell;    
  }  

  return alpha;
}

template <typename T>
T ConvexBarrierSolver<T>::DoNewtonLineSearch(const State& state,
                                       const VectorX<T>& dgamma, ConvexBarrierSolverStats* stats, SparseMatrix<T>*) const {
  using std::abs;
  const int nc = pre_proc_data_.nc;
  const int nc3 = 3 * nc;

  State state_alpha(state);   
  VectorX<T> ell_gradient(nc3);

  const auto& gamma0 = state.gamma();
  auto& gamma_alpha = state_alpha.mutable_gamma();

  const T ell0 = CalcUnconstrainedCostAndGradients(state, &ell_gradient);
  const T dell_dalpha0 = ell_gradient.dot(dgamma);
  PRINT_VAR(dell_dalpha0);

  T alpha = 1.0;
  gamma_alpha = gamma0 + alpha * dgamma;
  T ell_alpha = CalcUnconstrainedCostAndGradients(state_alpha, &ell_gradient);
  T dell_dalpha = ell_gradient.dot(dgamma);

  // The minimum is beyond this ray. Skip LS and return alpha=1.0.
  if (dell_dalpha < 0) {
    return 1.0;
  }

  T alpha_left = 0.0;
  T alpha_right = 1.0;

  // I'll essentially use bi-section, choosing the left or right interval based
  // on the gradient.
  for (int ils = 0; ils < parameters_.max_ls_iters; ++ils) {
    stats->ls_iterations++;
    const T dalpha = (alpha_right - alpha_left);
    alpha = 0.5 * (alpha_left + alpha_right);
    if (dalpha < 1.0e-3) return alpha;
    PRINT_VAR(alpha);
    gamma_alpha = gamma0 + alpha * dgamma;
    ell_alpha = CalcUnconstrainedCostAndGradients(state_alpha, &ell_gradient);
    dell_dalpha = ell_gradient.dot(dgamma);
    PRINT_VAR(ell_alpha);

    if (dell_dalpha > 0)
      alpha_right = alpha;
    else
      alpha_left = alpha;

  }
  return alpha;

#if 0
  SparseMatrix<T>& ell_hessian = *ell_hessian_ptr;

  T alpha = 1.0;
  gamma_alpha = gamma0 + alpha * dgamma;
  T ell_alpha = CalcUnconstrainedCostAndGradients(state_alpha, &ell_gradient,
                                                  &ell_hessian);
  T dell_dalpha = ell_gradient.dot(dgamma);
  T d2ell_dalpha2 = dgamma.dot(ell_hessian * dgamma);  

  // ell should be strictly convex in this feasible ray.
  DRAKE_DEMAND(d2ell_dalpha2 > 0);

  // The minimum is beyond this ray. Skip LS and return alpha=1.0.
  if (dell_dalpha < 0) {
    return 1.0;
  }

#if 0
  std::ofstream file("log.dat");
  for (T a(0); a < 1.0; a += 0.00001) {
    gamma_alpha = gamma0 + a * dgamma;
    T cost = ell_alpha = CalcUnconstrainedCostAndGradients(
        state_alpha, &ell_gradient, &ell_hessian);
    dell_dalpha = ell_gradient.dot(dgamma);
    d2ell_dalpha2 = dgamma.dot(ell_hessian * dgamma);
    std::string buff = fmt::format("{:18.6g} {:18.6g} {:18.6g} {:18.6g}\n", a,
                                   cost, dell_dalpha, d2ell_dalpha2);
    file << buff;
  }
  file.close();
#endif  

  T ell_prev = ell_alpha;
  alpha = 1.0 - dell_dalpha / d2ell_dalpha2;
  for (int ils = 0; ils < parameters_.max_ls_iters; ++ils) {
    stats->ls_iterations++;

    PRINT_VAR(alpha);

    gamma_alpha = gamma0 + alpha * dgamma;
    ell_alpha = CalcUnconstrainedCostAndGradients(state_alpha, &ell_gradient,
                                                    &ell_hessian);
    dell_dalpha = ell_gradient.dot(dgamma);
    d2ell_dalpha2 = dgamma.dot(ell_hessian * dgamma);

    PRINT_VAR(d2ell_dalpha2);

    //PRINT_VAR(alpha);
    //PRINT_VAR(ell_alpha);
    //PRINT_VAR(ell_prev);
    //DRAKE_DEMAND(ell_alpha < ell_prev);

    // The unconstrained cost should decrease in the direction of dgamma.
    //DRAKE_DEMAND(dell_dalpha < 0);

    // ell should be strictly convex in this feasible ray.
    DRAKE_DEMAND(d2ell_dalpha2 > 0);

    const T delta_alpha = -dell_dalpha / d2ell_dalpha2;
    alpha += delta_alpha;

    if (abs(delta_alpha) < 0.001) return alpha;

    ell_prev = ell_alpha;
    (void)ell_prev;
  }

  return alpha;
#endif  
}

template <typename T>
void ConvexBarrierSolver<T>::EstimateKappa(const VectorX<T>& gamma,
                                           const VectorX<T>& vc,
                                           VectorX<T>* kappa) const {
  const auto& N = pre_proc_data_.N;
  const auto& r = pre_proc_data_.r;
  const auto& mi = pre_proc_data_.mi;
  const int nc = pre_proc_data_.nc;

  const VectorX<T> N_times_gamma = N * gamma;
  const T ell = gamma.dot(0.5 * N_times_gamma + r);

  PRINT_VAR(gamma.dot(N_times_gamma));

  const T M = std::accumulate(&mi[0], &mi[nc-1], 0.0);

  T sum_vg = 0;
  for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
    const T v_norm = vc.template segment<3>(ic3).norm();
    const T g_norm = gamma.template segment<3>(ic3).norm();
    (*kappa)[ic] = parameters_.kappa_epsilon * v_norm * g_norm;
    sum_vg += (v_norm * g_norm);
  }

  PRINT_VAR(sum_vg);
  PRINT_VAR(ell);
  PRINT_VAR(M);
  PRINT_VAR(kappa->transpose());
}

template <typename T>
void ConvexBarrierSolver<T>::EstimateKappaGivenAccuracy(
    bool first_time, const T& beta, const VectorX<T>& gamma_id,
    const VectorX<T>& vc, VectorX<T>* kappa) const {
  const auto& mu = pre_proc_data_.mu;
  const auto& Rt = pre_proc_data_.Rt;
  const auto& R = pre_proc_data_.R;
  const auto& vc_stab = pre_proc_data_.vc_stab;
  const int nc = pre_proc_data_.nc;

  using std::max;
  using std::min;
  using std::sqrt;

  if (first_time) {
    // Estimate a value of gamma that will always lead to a non-zero value.
    // We assume pentration, i.e. b < 0.
    const VectorX<T> y_prime = vc_stab.cwiseQuotient(R);

    // Slip velocity scale, [m/s].
    const T u0 = 1.0;

    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      const T gamma_estimate = y_prime.template segment<3>(ic3).norm();
      // t is the error estimate.
      const T t = parameters_.rel_tolerance * gamma_estimate;

      // Estimate a large value of Delta, conservative to estimate kappa.
      const T Delta = u0 / Rt(ic);
      const T z = (2 * t + Delta) * (2 * t + Delta) - Delta * Delta;
      const T kappa_estimate = Rt(ic) * z / 4.0;

      (*kappa)(ic) = kappa_estimate;
    }
  } else {
    const VectorX<T> y = -(vc - vc_stab).cwiseQuotient(R);
    for (int ic = 0, ic3 = 0; ic < nc; ic++, ic3 += 3) {
      const auto y_ic = y.template segment<3>(ic3);
      const T gamma_norm = gamma_id.template segment<3>(ic3).norm();

      // t is the difference between gamma_id and gamma_kappa.
      const T t = parameters_.rel_tolerance * gamma_norm;

      const T delta = -CalcConeSdf(mu(ic), y_ic);

      const T z =
          (2.0 * t + abs(delta)) * (2.0 * t + abs(delta)) - delta * delta;
      PRINT_VAR(t);
      PRINT_VAR(z);

      const T kappa_prev = (*kappa)(ic);
      T kappa_next = 0.25 * Rt(ic) * z;

      PRINT_VAR(kappa_next);

      if (first_time) {
        // flag zero values.
        if (gamma_norm < 1.0e-15) {
          (*kappa)(ic) = -1;
        } else {
          (*kappa)(ic) = kappa_next;
        }
      } else {
        kappa_next = min(kappa_next, 10.0 * kappa_prev);

        // We only let it decrease when the previous iteration was "solid",
        // meaning we were able to take a "long" step.
        if (kappa_next > kappa_prev || beta > 0.5) {
          kappa_next = max(kappa_next, 0.1 * kappa_prev);
          (*kappa)(ic) = kappa_next;
          //PRINT_VAR(kappa_next);
        }
      }
    }
  }

#if 0
  const T max_kappa = kappa->maxCoeff();
  PRINT_VAR(max_kappa);
  for (int ic = 0; ic < nc; ++ic) {
    if ((*kappa)(ic) < 0) {
      (*kappa)(ic) = max_kappa;
    }
  }
#endif  
}

template <typename T>
void ConvexBarrierSolver<T>::CalcInitialCondition(const VectorX<T>& vc,
                                                  VectorX<T>* gamma) const {
  using std::abs;
  using std::max;
  using std::sqrt;
  const VectorX<T>& Rn = pre_proc_data_.Rn;
  const VectorX<T>& Rt = pre_proc_data_.Rt;
  const VectorX<T>& mu = pre_proc_data_.mu;
  const VectorX<T>& mi = pre_proc_data_.mi;
  const VectorX<T>& mu_tilde = pre_proc_data_.mu_tilde;
  const VectorX<T>& vc_stab = pre_proc_data_.vc_stab;
  const VectorX<T>& vc_star = pre_proc_data_.vc_star;

  const double eps_rel = parameters_.rel_tolerance;
  const double eps_abs = parameters_.abs_tolerance;

  for (int ic = 0, ic3 = 0; ic < num_contacts(); ic++, ic3 += 3) {
    const auto vc_star_ic = vc_star.template segment<3>(ic3);
    const auto vc_stab_ic = vc_stab.template segment<3>(ic3);
    const T vel_scale = max(vc_star_ic.norm(), vc_stab_ic.norm());
    const T gamma_scale = mi(ic) * vel_scale;
    PRINT_VAR(vel_scale);
    PRINT_VAR(gamma_scale);
    const T eps_gamma = mi(ic) * (eps_abs + vel_scale * eps_rel);
    PRINT_VAR(eps_gamma);

    // y = -R⁻¹⋅vc_tilde_ic
    auto vc_ic = vc.template segment<3>(ic3);
    const Vector3<T> vc_tilde_ic = vc_ic - vc_stab_ic;
    const Vector3<T> y_ic(-vc_tilde_ic(0) / Rt(ic), -vc_tilde_ic(1) / Rt(ic),
                          -vc_tilde_ic(2) / Rn(ic));
    Vector3<T> fperp;  // The direction of the projection.
    const Vector3<T> gamma_id =
        ProjectImpulse(mu(ic), mu_tilde(ic), Rt(ic) / Rn(ic), y_ic, &fperp);

    const T gamma_id_norm = gamma_id.norm();

    auto gamma_eps = gamma->template segment<3>(ic3);

    if (gamma_id_norm > eps_gamma) {
      gamma_eps = gamma_id - eps_gamma * fperp;
      const auto& beta_id = gamma_id.template head<2>();
      const auto& beta_eps = gamma_eps.template head<2>();
      if (beta_id.dot(beta_eps) < 0) gamma_eps.template head<2>().setZero();
    } else {
      // When inside polar cone we have gamma_id = 0.
      gamma_eps = eps_gamma * Vector3<T>::UnitZ();
    }
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::ConvexBarrierSolver<
    double>;
