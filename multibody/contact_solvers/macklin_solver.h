#pragma once

#include <memory>
#include <vector>

#include <iostream>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/point_contact_data.h"
#include "drake/multibody/contact_solvers/scratch_workspace.h"
#include "drake/multibody/contact_solvers/system_dynamics_data.h"
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {

struct MacklinSolverParameters {
  // [Macklin et al., 2019] uses Gcᵀ instead of the original Jcᵀ in the
  // equations of momentum to obtain a symmetric system of equations. However,
  // we can still use Jc and obtain a symmetric system. Below,
  //   macklin_jacobian = true corresponds to using Gc.
  //   macklin_jacobian = false corresponds to using Jc.
  double macklin_jacobian{true};

  double stiction_tolerance{1.0e-5};  // vs in [m/s].
  // stabilization parameter, tau_rigid = alpha_stab * dt.
  // With alpha_stab = 1.0 we have the position based constraint as Anitescu.
  // If alpha_stab < 0, we set v_stab = 0.
  double alpha_stab{-1.0};
  double fb_velocity_scale{1.0e-7};  // for "soft" FB.
  // Relaxation for the initial centering step (first outer iteration).
  double relaxation{0.7};
  // if max_ls_iters = 0 and limit_to_feasible_values = false, we have a fixed
  // relaxation scheme weth value "relaxation".
  int max_ls_iters = 12;

  // Outer loop:
  int max_iters{400};
  double absolute_tolerance{1.0e-6};  // Absolute tolerance in [m/s].
  double relative_tolerance{1e-4};    // Relative tolerance.

  // TODO: Consider other convergence criteria. What does FClib do?

  // Tolerance relative to the value of stiction_tolerance.
  // On convergence, the final value of m will be:
  //   m = complementary_slackness_tolerance * stiction_tolerance.
  // TODO: investigate other values, but it'd seem a value close to vs should
  // work ok.
  // double complementary_slackness_tolerance{1.0};
};

// Metrics used to determine convergence.
// They all have units of velocity, [m/s] and thus are comparable.
struct ErrorMetrics {
  double normal_slackness_error{-1.0};
  double tangential_slackness_error{-1.0};
  double mdp_error{-1.0};
  double momentum_error{-1.0};
};

struct MacklinSolverIterationStats {
  int iterations{0};
  std::vector<ErrorMetrics> iteration_errors;
};

template <typename T>
class MacklinSolver : public ContactSolver<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MacklinSolver);

  // "cached quantities". They are all function of "state".
  struct Cache {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache);

    Cache() = default;

    // Contact velocities are all updated at once.
    bool contact_velocities_dirty{true};  // It depends on v only.
    VectorX<T> vc;                        // 3 * nc
    VectorX<T> vn;                        // nc
    VectorX<T> vt;                        // 2* nc

    // The nc normal constraints are written as:
    //   gnᵢ(v, π) = fFB(cₙᵢ(v, π), π) = 0.
    // with:
    //   cₙᵢ(v, π) = vₙᵢ − vₛᵢ + Rₙᵢ πᵢ
    // Its Jacobian is defined as:
    //   Gₙ = ∂gₙ(v, π)/∂v = diag(∂gₙᵢ(v, π)/∂vₙᵢ)⋅Jₙ
    bool normal_constraints_dirty{true};  // It depends on v and gamma.
    VectorX<T> Rn;                        // nc
    VectorX<T> cn;                        // nc
    VectorX<T> DcnDvn;  // DcnDvn(i) = ∂gₙᵢ(v, π)/∂vₙᵢ
    VectorX<T> gn;      // gn(i) = fFB(cₙ(i), pi(i))
    VectorX<T> DgnDvn;  // Diagonal matrix, of size nc.
    VectorX<T> DgnDpi;  // Diagonal matrix, of size nc.

    // The MDP constraints are written as:
    //   μ π vₜ + λ β = 0
    // Using the fixed-point iteration scheme in [Macklin et al. 2019] we write:
    //    vₜ + Wmdp β = 0
    bool max_dissipation_constraints_dirty{
        true};  // It depends on lambda and gamma.
    VectorX<T> dlambda;
    VectorX<T> dbeta_norm;
    VectorX<T> Wmdp;
    VectorX<T> gt;

    // The Delassus operator is defined as W = Jc⋅A⁻¹⋅Jcᵀ.
    // We defined the "tilde" Delassus operator as:
    // W = Gc(v)⋅A⁻¹⋅Gc(v)ᵀ.
    bool delassus_dirty{true};
    Eigen::SparseMatrix<T> Wtilde;  // It depends on v and gamma.

    void Resize(int nv, int nc) {
      vc.resize(3 * nc);
      vn.resize(nc);
      vt.resize(2 * nc);
      Rn.resize(nc);
      cn.resize(nc);
      DcnDvn.resize(nc);
      gn.resize(nc);
      DgnDvn.resize(nc);
      DgnDpi.resize(nc);
      dlambda.resize(nc);
      dbeta_norm.resize(nc);
      Wmdp.resize(nc);
      gt.resize(2 * nc);
      Wtilde.resize(3 * nc, 3 * nc);
    }
  };

  // The state of the iteration.
  class State {
   public:
    // N.B. We do want to copy the cache as is during assignment, including
    // cached quantities.
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);

    State() = default;

    State(int nv, int nc) { Resize(nv, nc); }

    void Resize(int nv, int nc) {
      v_.resize(nv);
      gamma_.resize(3 * nc);
      lambda_.resize(nc);
      cache_.Resize(nv, nc);
    }

    void Set(const VectorX<T>& v, const VectorX<T>& gamma, const T& m) {
      SetVelocities(v);
      SetImpulses(gamma);
    }

    void SetVelocities(const VectorX<T>& v) { mutable_v() = v; }

    void SetImpulses(const VectorX<T>& g) { mutable_gamma() = g; }

    const VectorX<T>& v() const { return v_; }
    const VectorX<T>& gamma() const { return gamma_; }
    const Cache& cache() const { return cache_; }
    Cache& mutable_cache() const { return cache_; }
    const VectorX<T>& lambda() const { return lambda_; }

    VectorX<T>& mutable_v() {
      cache_.contact_velocities_dirty = true;
      cache_.normal_constraints_dirty = true;
      cache_.delassus_dirty = true;
      return v_;
    }

    VectorX<T>& mutable_gamma() {
      cache_.normal_constraints_dirty = true;
      cache_.delassus_dirty = true;
      return gamma_;
    }

    VectorX<T>& mutable_lambda() {
      cache_.max_dissipation_constraints_dirty = true;
      return lambda_;
    }

   private:
    VectorX<T> v_;       // nv
    VectorX<T> gamma_;   // 3 * nc.
    VectorX<T> lambda_;  // nc

    // We "cache" computations that depend on the state of the iteration.
    mutable Cache cache_;
  };

  MacklinSolver(int nv, int nc) : scratch_workspace_(nv, nc, 64) {}

  void SetSystemDynamicsData(const SystemDynamicsData<T>* data) final;

  void SetPointContactData(const PointContactData<T>* data) final;

  int num_contacts() const final { return contact_data_->num_contacts(); }

  int num_velocities() const final { return dynamics_data_->num_velocities(); }

  ContactSolverResult SolveWithGuess(const VectorX<T>& v_guess) final;
  ContactSolverResult SolveWithGuess(const State& state_guess);

  void set_parameters(const MacklinSolverParameters& p) { parameters_ = p; }

  const MacklinSolverIterationStats& get_stats() const { return stats_; }

  // TODO: remove, we have COntactSolver::CopyNormalImpulses().
  VectorX<T> CopyNormalForces() const {
    GrantScratchWorkspaceAccess access(scratch_workspace_);
    auto& pi = access.xn_sized_vector();
    ExtractNormal(state_.gamma(), &pi);
    return pi;
  }

  const State& get_state() const { return state_; }

 private:
  // All this data must remain const after the call to PreProcessData().
  struct PreProcessedData {
    Eigen::SparseMatrix<T> W;
    VectorX<T> vc_star;
    // Norm of the 3x3 block diagonal block of matrix W, of size nc.
    VectorX<T> Wii_norm;
    VectorX<T> vn0;      // Initial normal velocity.
    VectorX<T> vn_stab;  // Normal stabilization velocity.
    void Resize(int nv, int nc) {
      W.resize(3 * nc, 3 * nc);
      vc_star.resize(3 * nc);
      Wii_norm.resize(nc);
      vn0.resize(nc);
      vn_stab.resize(nc);
    }
  };

  // DgDvc is a diagonal matrix with entries DgDvc = [1, 1, DgnDvn] for each
  // contact point.
  // yc = DgDvc * xc, is O(nc).
  void MultiplyBy_DgDvc(const State& s, const VectorX<T>& xc,
                        VectorX<T>* yc) const {
    DRAKE_DEMAND(3 * num_contacts() == xc.size());
    DRAKE_DEMAND(3 * num_contacts() == yc->size());
    const VectorX<T>& DgnDvn = EvalDgnDvn(s);
    for (int ic = 0, ic3 = 0; ic < num_contacts(); ic++, ic3 += 3) {
      // DgcDvc_ic = [1; 1; DgnDvn(ic)], therefore:
      //   yc_ic = DgcDvc_ic * xc_ic = [xc_ic(0); xc_ic(1); xc_ic(2)*DgnDvn(ic)]
      yc->template segment<3>(ic3) = xc.template segment<3>(ic3);
      (*yc)(ic3 + 2) *= DgnDvn(ic);
    }
  }

  void MultiplyBy_DgDvc(const State& s, const Eigen::SparseVector<T>& xc,
                        Eigen::SparseVector<T>* yc) const {
    DRAKE_DEMAND(3 * num_contacts() == xc.size());
    DRAKE_DEMAND(3 * num_contacts() == yc->size());
    const VectorX<T>& DgnDvn = EvalDgnDvn(s);

    // Copy xc, with the same sparsity pattern.
    (*yc) = xc;
    for (int k = 0; k < yc->outerSize(); ++k) {
      for (typename Eigen::SparseVector<T>::InnerIterator it(*yc, k); it;
           ++it) {
        const int i = it.row();
        const int j = it.col();
        DRAKE_DEMAND(j == 0);
        const int icoo = i % 3;  // coordinate index, icoo = 0, 1 or, 2
        if (icoo == 2) {         // Only normal components get modified.
          const int ic = i / 3;
          it.valueRef() *= DgnDvn(ic);
        }
      }
    }
  }

  // Gc = DgDvc * Jc
  // Then yc = Gc * v = DgDvc * Jc * v
  void MultiplyByGc(const State& s,
                    const Eigen::Ref<const Eigen::SparseVector<T>>& v,
                    Eigen::SparseVector<T>* vc_tilde) const {
    DRAKE_DEMAND(vc_tilde->size() == 3 * num_contacts());
    get_Jc().Multiply(v, vc_tilde);              // vc_tilde = Jc * v
    MultiplyBy_DgDvc(s, *vc_tilde, &*vc_tilde);  // vc_tilde = Gc * v
  }

  void MultiplyByGc(const State& s, const Eigen::Ref<const VectorX<T>>& v,
                    VectorX<T>* vc_tilde) const {
    DRAKE_DEMAND(vc_tilde->size() == 3 * num_contacts());
    get_Jc().Multiply(v, vc_tilde);              // vc_tilde = Jc * v
    MultiplyBy_DgDvc(s, *vc_tilde, &*vc_tilde);  // vc_tilde = Gc * v
  }

  // tau_c = Gc(s)^T * fc = (DgnDvn*Jc)^T * fc = Jc^T * DgnDvn * fc
  template <class DenseOrSparseVector>
  void MultiplyByGcTranspose(const State& s, const DenseOrSparseVector& fc,
                             DenseOrSparseVector* tau_c) const {
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    // auto& fc_tilde = access.xc_sized_vector();
    DenseOrSparseVector fc_tilde(3 * num_contacts());
    MultiplyBy_DgDvc(s, fc, &fc_tilde);  // fc_tilde = DgDvc * fc
    get_Jc().MultiplyByTranspose(fc_tilde,
                                 &*tau_c);  // tau_c = JcT * DgDvc * fc
  }

  // Define linear operator objects for Gc and GcT.
  class GcOperator : public LinearOperator<T> {
   public:
    GcOperator(const MacklinSolver<T>* solver, const State* state)
        : LinearOperator<T>("Gc"), solver_(solver), state_(state){};

    ~GcOperator() {}

    int rows() const final { return 3 * solver_->num_contacts(); }
    int cols() const final { return solver_->num_velocities(); }

   private:
    /// Performs y = Gc*x.
    void DoMultiply(const Eigen::Ref<const Eigen::SparseVector<T>>& x,
                    Eigen::SparseVector<T>* y) const final {
      DRAKE_DEMAND(y != nullptr);
      DRAKE_DEMAND(x.size() == this->cols());
      DRAKE_DEMAND(y->size() == this->rows());
      solver_->MultiplyByGc(*state_, x, y);
    }

    /// Performs y = Gc*x.
    void DoMultiply(const Eigen::Ref<const VectorX<T>>& x,
                    VectorX<T>* y) const final {
      DRAKE_DEMAND(y != nullptr);
      DRAKE_DEMAND(x.size() == this->cols());
      DRAKE_DEMAND(y->size() == this->rows());
      solver_->MultiplyByGc(*state_, x, y);
    }

    /// Performs y = GcT*x.
    void DoMultiplyByTranspose(const Eigen::SparseVector<T>& x,
                               Eigen::SparseVector<T>* y) const final {
      DRAKE_DEMAND(y != nullptr);
      DRAKE_DEMAND(x.size() == this->cols());
      DRAKE_DEMAND(y->size() == this->rows());
      solver_->MultiplyByGcTranspose(*state_, x, y);
    }

    /// Performs y = GcT*x.
    void DoMultiplyByTranspose(const VectorX<T>& x, VectorX<T>* y) const final {
      DRAKE_DEMAND(y != nullptr);
      DRAKE_DEMAND(x.size() == this->cols());
      DRAKE_DEMAND(y->size() == this->rows());
      solver_->MultiplyByGcTranspose(*state_, x, y);
    }
    const MacklinSolver<T>* solver_{nullptr};
    const State* state_{nullptr};
  };

  // Compute residual of the generalized velocities:
  //   Fv = v - v_star - Mi * Jvᵀ * gamma.
  // where Jv = Gc if parameters_.macklin_jacobian = true and,
  // where Jv = Jc if parameters_.macklin_jacobian = false.
  void CalcVelocitiesResidual(const State& s, VectorX<T>* Fv) const {
    const auto& v = s.v();
    const auto& gamma = s.gamma();
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& JvT_gamma = access.v_sized_vector();
    auto& Ai_JvT_gamma = access.v_sized_vector();

    // JvT_gamma = Jᵀ * gamma.
    if (parameters_.macklin_jacobian) {
      // JvT_gamma = Gc(s)ᵀ * gamma.
      MultiplyByGc(s, gamma, &JvT_gamma);
    } else {
      // JvT_gamma = Jcᵀ * gamma.
      get_Jc().MultiplyByTranspose(gamma, &JvT_gamma);
    }

    get_Ainv().Multiply(JvT_gamma, &Ai_JvT_gamma);
    *Fv = v - get_v_star() - Ai_JvT_gamma;
  }

  // Given:
  //   v: contact velocities vector in m/s, of size nc.
  //   g: impulses vector in Ns, of size nc.
  // This method computes a merit on the complementarity slackness between v
  // and g as:
  //   slackness_error = ‖min(vᵢ, gᵢ/rᵢ)‖∞.
  // That is, we scale g to have units of velocity in gᵢ/rᵢ, compute the
  // minmap between vᵢ and gᵢ/rᵢ, and take the max norm.
  // Notice that slackness_error = 0 iff 0 ≤ v ⊥ g ≥ 0.
  T CalcVelocityScaledComplementaritySlacknessError(const VectorX<T>& v,
                                                    const VectorX<T>& g) const {
    DRAKE_DEMAND(v.size() == num_contacts());
    DRAKE_DEMAND(g.size() == num_contacts());
    const auto& r = pre_proc_data_.Wii_norm;
    const int nc = num_contacts();
    using std::abs;
    using std::max;
    using std::min;
    T error = 0.0;
    for (int ic = 0; ic < nc; ++ic) {
      const T error_ic = abs(min(v(ic), g(ic) / r(ic)));
      error = max(error, error_ic);
    }
    return error;
  }

  // Computes the velocity scaled complementarity slackness error between cn and
  // pi, cn = vn - v_stab + Rn * pi.
  T CalcNormalComplementaritySlacknessError(const State& s) const {
    const auto& cn = EvalCn(s);
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& pi = access.xn_sized_vector();
    ExtractNormal(s.gamma(), &pi);
    return CalcVelocityScaledComplementaritySlacknessError(cn, pi);
  }

  // Computes the velocity scaled complementarity slackness error between the
  // slip velocity ‖vₜ‖ and δ = μπ − ‖β‖.
  T CalcTangentialComplementaritySlacknessError(const State& s) const {
    const auto& vt = EvalVt(s);
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& pi = access.xn_sized_vector();
    ExtractNormal(s.gamma(), &pi);
    auto& beta = access.xt_sized_vector();
    ExtractTangent(s.gamma(), &beta);
    const auto& mu = get_mu();
    auto& vt_norm = access.xn_sized_vector();
    auto& delta = access.xn_sized_vector();
    for (int ic = 0, ic2 = 0; ic < num_contacts(); ++ic, ic2 += 2) {
      const auto vt_ic = vt.template segment<2>(ic2);
      const auto beta_ic = beta.template segment<2>(ic2);
      vt_norm(ic) = vt_ic.norm();
      delta(ic) = mu(ic) * pi(ic) - beta_ic.norm();
    }
    return CalcVelocityScaledComplementaritySlacknessError(vt_norm, delta);
  }

  // Computes an error metric on the MDP defined as:
  //   error_mdp = ‖μπ⋅vₜ + ‖vₜ‖⋅β‖∞
  T CalcMaximumDissipationPrincipleError(const State& s) const {
    using std::abs;
    using std::max;
    const auto& vt = EvalVt(s);
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& pi = access.xn_sized_vector();
    ExtractNormal(s.gamma(), &pi);
    auto& beta = access.xt_sized_vector();
    ExtractTangent(s.gamma(), &beta);
    const auto& mu = get_mu();
    T error = 0.0;
    for (int ic = 0, ic2 = 0; ic < num_contacts(); ++ic, ic2 += 2) {
      const auto vt_ic = vt.template segment<2>(ic2);
      const auto beta_ic = beta.template segment<2>(ic2);
      const T error_ic = (mu(ic) * pi(ic) * vt_ic + vt_ic.norm() * beta).norm();
      error = max(error, error_ic);
    }
    return error;
  }

  // Computes the error on the momentum equations projected onto the contact
  // space.
  // Defined as error_mom = ‖Jc⋅Fv‖∞.
  T CalcMomentumError(const State& s) const {
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& Fv = access.v_sized_vector();
    CalcVelocitiesResidual(s, &Fv);
    auto& Fvc = access.xc_sized_vector();
    get_Jc().Multiply(Fv, &Fvc);
    return Fvc.template lpNorm<Eigen::Infinity>();
  }

  bool CheckConvergenceCriteria(const State& s, ErrorMetrics* errors) const {
    errors->normal_slackness_error =
        ExtractDoubleOrThrow(CalcNormalComplementaritySlacknessError(s));
    errors->tangential_slackness_error =
        ExtractDoubleOrThrow(CalcTangentialComplementaritySlacknessError(s));
    errors->mdp_error =
        ExtractDoubleOrThrow(CalcMaximumDissipationPrincipleError(s));
    errors->momentum_error = ExtractDoubleOrThrow(CalcMomentumError(s));
    const VectorX<T>& vc = EvalVc(s);
    const double vel_scale = ExtractDoubleOrThrow(vc.norm());
    auto within_error_bounds = [& p = parameters_, &vel_scale](double error) {
      const double bounds =
          p.absolute_tolerance + p.relative_tolerance * vel_scale;
      return error < bounds;
    };
    return within_error_bounds(errors->normal_slackness_error) &&
           within_error_bounds(errors->tangential_slackness_error) &&
           within_error_bounds(errors->mdp_error) &&
           within_error_bounds(errors->momentum_error);
  }

  void CalcNormalStabilizationVelocity(const T& dt, double alpha_stab,
                                       const VectorX<T>& vn0,
                                       const VectorX<T>& phi0,
                                       VectorX<T>* vn_stab);

  // Update vc.
  void UpdateContactVelocities(const VectorX<T>& v, VectorX<T>* vc,
                               VectorX<T>* vn, VectorX<T>* vt) const;

  // No-op if contact velocities are up-to-date with the state s.
  // Otherwise it re-computes the contact velocities.
  void ValidateContactVelocitiesCache(const State& s) const {
    Cache& c = s.mutable_cache();
    if (c.contact_velocities_dirty) {
      const auto& v = s.v();
      auto& vc = c.vc;
      auto& vn = c.vn;
      auto& vt = c.vt;
      UpdateContactVelocities(v, &vc, &vn, &vt);
      c.contact_velocities_dirty = false;
    }
  }

  const VectorX<T>& EvalVc(const State& s) const {
    ValidateContactVelocitiesCache(s);
    return s.cache().vc;
  }

  const VectorX<T>& EvalVn(const State& s) const {
    ValidateContactVelocitiesCache(s);
    return s.cache().vn;
  }

  const VectorX<T>& EvalVt(const State& s) const {
    ValidateContactVelocitiesCache(s);
    return s.cache().vt;
  }

  const VectorX<T>& EvalCn(const State& s) const {
    ValidateNormalConstraintsCache(s);
    return s.cache().cn;
  }

  const VectorX<T>& Eval_gn(const State& s) const {
    ValidateNormalConstraintsCache(s);
    return s.cache().gn;
  }

  const VectorX<T>& Eval_gt(const State& s) const {
    ValidateMaxDissipationConstraintsCache(s);
    return s.cache().gt;
  }

  const VectorX<T>& Eval_Wmdp(const State& s) const {
    ValidateMaxDissipationConstraintsCache(s);
    return s.cache().Wmdp;
  }

  const VectorX<T>& EvalDgnDvn(const State& s) const {
    ValidateNormalConstraintsCache(s);
    return s.cache().DgnDvn;
  }

  const VectorX<T>& Eval_DgnDpi(const State& s) const {
    ValidateNormalConstraintsCache(s);
    return s.cache().DgnDpi;
  }

  const Eigen::SparseMatrix<T>& EvalWtilde(const State& s) const {
    Cache& c = s.mutable_cache();
    if (c.delassus_dirty) {
      Eigen::SparseMatrix<T>& Wtilde = c.Wtilde;
      // Wtilde = diag(DgcDvc) * W * R;
      // Where
      //   R = diag(DgcDvc) if parameters_.macklin_jacobian = true and,
      //   R = Id if parameters_.macklin_jacobian = false.
      //
      // Recall however the structure of DgcDvc for the i-th contact:
      //   DgcDvc_ic = [1; 1; DgnDvn_ic]
      // Therefore:
      //   Wtilde_ij = W_ij * DgcDvc_ic(icoo) * DgcDvc_jc(icoo),
      //   iff i = 3 * ic + icoo and j = 3 * jc + jcoo,
      // with 0 <= ic, jc < nc, contact point indexes and
      // 0 <= icoo, jcoo < 3, coordinates indexes.
      // This operation can be further simplified noticing that
      // DgcDvc_ic(0) = DgcDvc_ic(1) = 1. Thus only DgnDvn is needed.
      //
      // From above, we conclude this operation is O(2/3*nnz(W)).
      // We perform it explicitly by scanning the non-zeros of W.
      const VectorX<T>& DgnDvn = EvalDgnDvn(s);
      Wtilde = pre_proc_data_.W;
      for (int k = 0; k < Wtilde.outerSize(); ++k) {
        for (typename Eigen::SparseMatrix<T>::InnerIterator it(Wtilde, k); it;
             ++it) {
          const int i = it.row();
          const int icoo = i % 3;
          const int j = it.col();
          const int jcoo = j % 3;
          if (icoo == 2) {
            int ic = i / 3;
            it.valueRef() *= DgnDvn(ic);
          }
          if (jcoo == 2 && parameters_.macklin_jacobian) {
            int jc = j / 3;
            it.valueRef() *= DgnDvn(jc);
          }
        }
      }
      c.delassus_dirty = false;
    }
    return c.Wtilde;
  }

  // We use a soft norm with epsilon_square.
  // x, y should have the same units.
  // epsilon_squared should have units of x (or y)  squared.
  // if grad_phi = nullptr gradient is not computed.
  // TODO: revisit the functional form of the "softened" version.
  // since phi(0,0) = -epsilon. Is this what we want? do I want to make
  // phi(0,0) = 0? is there a better way?
  static T CalcFischerBurmeister(const T& x, const T& y, double epsilon_squared,
                                 EigenPtr<Vector2<T>> grad_phi);

  void CalcNormalConstraintResidual(const VectorX<T>& vn, const VectorX<T>& pi,
                                    const T& dt, VectorX<T>* cn, VectorX<T>* gn,
                                    VectorX<T>* Rn = nullptr,
                                    VectorX<T>* DcnDvn = nullptr,
                                    VectorX<T>* DgnDvn = nullptr,
                                    VectorX<T>* DgnDpi = nullptr) const;

  void CalcMaxDissipationConstraintResidual(
      const VectorX<T>& vt, const VectorX<T>& lambda, const VectorX<T>& beta,
      const VectorX<T>& pi, VectorX<T>* dlambda, VectorX<T>* dbeta_norm,
      VectorX<T>* W, VectorX<T>* gt) const;

  // Update normal constraints cached computations to state s.
  // No-op if already up-to-date with state s.
  void ValidateNormalConstraintsCache(const State& s) const {
    Cache& c = s.mutable_cache();
    if (c.normal_constraints_dirty) {
      ValidateContactVelocitiesCache(s);
      GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
      auto& pi = access.xn_sized_vector();
      ExtractNormal(s.gamma(), &pi);
      CalcNormalConstraintResidual(c.vn, pi, get_dt(), &c.cn, &c.gn, &c.Rn,
                                   &c.DcnDvn, &c.DgnDvn, &c.DgnDpi);
      c.normal_constraints_dirty = false;
    }
  }

  void ValidateMaxDissipationConstraintsCache(const State& s) const {
    Cache& c = s.mutable_cache();
    if (c.max_dissipation_constraints_dirty) {
      ValidateContactVelocitiesCache(s);
      GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
      const auto& vt = c.vt;
      const auto& lambda = s.lambda();
      auto& beta = access.xt_sized_vector();
      auto& pi = access.xn_sized_vector();
      ExtractNormal(s.gamma(), &pi);
      ExtractTangent(s.gamma(), &beta);
      CalcMaxDissipationConstraintResidual(vt, lambda, beta, pi, &c.dlambda,
                                           &c.dbeta_norm, &c.Wmdp, &c.gt);
      c.max_dissipation_constraints_dirty = false;
    }
  }

  // TODO: extract line-search from this method.
  // T LimitNormalUpdate(const State& s_km, const State& s_kp,
  //                    int outer_iter) const;

  T CalcLineSearchParameter(const State& s_km, const State& s_kp) const;

  void PreProcessData();

  // Quick accessors to problem data.
  const T& get_dt() const { return dynamics_data_->get_dt(); }
  const LinearOperator<T>& get_Jc() const { return contact_data_->get_Jc(); }
  const LinearOperator<T>& get_Ainv() const {
    return dynamics_data_->get_Ainv();
  }
  const VectorX<T>& get_v_star() const { return dynamics_data_->get_v_star(); }
  const VectorX<T>& get_v0() const { return dynamics_data_->get_v0(); }
  const VectorX<T>& get_phi0() const { return contact_data_->get_phi0(); }
  const VectorX<T>& get_stiffness() const {
    return contact_data_->get_stiffness();
  }
  const VectorX<T>& get_dissipation() const {
    return contact_data_->get_dissipation();
  }
  const VectorX<T>& get_mu() const { return contact_data_->get_mu(); }

  const SystemDynamicsData<T>* dynamics_data_{nullptr};
  const PointContactData<T>* contact_data_{nullptr};
  MacklinSolverParameters parameters_;

  // Pre-processed data must remain "const" after the first call to
  // PreProcessData() and it should only be modified by that method.
  PreProcessedData pre_proc_data_;

  // The state of the solver's iteration.
  State state_;

  // Workspace storing temporary variables.
  mutable ScratchWorkspace<T> scratch_workspace_;

  // Collect useful statistics.
  MacklinSolverIterationStats stats_;
};

}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
