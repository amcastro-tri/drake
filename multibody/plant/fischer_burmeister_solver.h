#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {

template <typename T>
class GrantScratchWorkspaceAccess;

template <typename T>
class ScratchWorkspace {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ScratchWorkspace);

  ScratchWorkspace(int nv, int nc, int max_vectors)
      : nv_(nv), nc_(nc), max_vectors_(max_vectors) {}

  /// Number of vectors of size 3 * nc currently being referenced.
  int num_xc_vectors() const { return xc_top_; }

  /// The current capacity for xc vectors. The maximum number of xc vectors so
  /// far requested.
  int xc_vectors_capacity() const { return xc_pool_.size(); }

 private:
  friend class GrantScratchWorkspaceAccess<T>;

  // Pushes a vector of size 3*nc into the stack and returns a reference to
  // it.
  VectorX<T>& push_xc_sized_vector() {    
    //PRINT_VAR(xc_top_);
    return AccessVectorPoolAndMaybeAllocate(xc_top_++, 3 * nc_, &xc_pool_);
  }

  // Pops `count`  xc sized vectors from the stack.
  void pop_xc_sized_vectors(int count) { xc_top_ -= count; }

  VectorX<T>& push_xn_sized_vector() {
    //PRINT_VAR(xn_top_);
    return AccessVectorPoolAndMaybeAllocate(xn_top_++, nc_, &xn_pool_);
  }
  void pop_xn_sized_vectors(int count) { xn_top_ -= count; }

  VectorX<T>& push_xt_sized_vector() {
    //PRINT_VAR(xt_top_);
    return AccessVectorPoolAndMaybeAllocate(xt_top_++, 2 * nc_, &xt_pool_);
  }
  void pop_xt_sized_vectors(int count) { xt_top_ -= count; }

  VectorX<T>& push_v_sized_vector() {
    //PRINT_VAR(v_top_);
    return AccessVectorPoolAndMaybeAllocate(v_top_++, nv_, &v_pool_);
  }
  void pop_v_sized_vectors(int count) { v_top_ -= count; }

  VectorX<T>& AccessVectorPoolAndMaybeAllocate(
      int i, int vector_size, std::vector<std::unique_ptr<VectorX<T>>>* pool) {
    if (i < static_cast<int>(pool->size())) {
      return *pool->at(i);
    } else {
      if (max_vectors_ == static_cast<int>(pool->size())) {
        throw std::runtime_error("Workspace reached maximum capacity.");
      }
      return *pool->emplace_back(std::make_unique<VectorX<T>>(vector_size));
    }
  }

  int nv_, nc_;
  int max_vectors_;
  int xc_top_{0};
  int xn_top_{0};
  int xt_top_{0};
  int v_top_{0};
  // We save pointers instead of the actual objects so that whenever a
  // .push_back() happens the actual vectors are not destroyed.
  std::vector<std::unique_ptr<VectorX<T>>>
      xc_pool_;  // pool of vectors of size 3*nc.
  std::vector<std::unique_ptr<VectorX<T>>>
      xn_pool_;  // pool of vectors of size nc.
  std::vector<std::unique_ptr<VectorX<T>>>
      xt_pool_;  // pool of vectors of size 2*nc.
  std::vector<std::unique_ptr<VectorX<T>>>
      v_pool_;  // pool of vectors of size nv.
};

template <typename T>
class GrantScratchWorkspaceAccess {
 public:
  /// Resets counter access for w.
  GrantScratchWorkspaceAccess(ScratchWorkspace<T>& w) : w_(w) {}

  /// Resets counter access for w.
  ~GrantScratchWorkspaceAccess() {
    w_.pop_xc_sized_vectors(xc_counter_);
    w_.pop_xn_sized_vectors(xn_counter_);
    w_.pop_xt_sized_vectors(xt_counter_);
    w_.pop_v_sized_vectors(v_counter_);
  }

  // Access an i-th pool for vectors of size 3*nc.
  VectorX<T>& xc_sized_vector() {
    ++xc_counter_;
    return w_.push_xc_sized_vector();
  }

  VectorX<T>& xn_sized_vector() {
    ++xn_counter_;
    return w_.push_xn_sized_vector();
  }

  VectorX<T>& xt_sized_vector() {
    ++xt_counter_;
    return w_.push_xt_sized_vector();
  }

  VectorX<T>& v_sized_vector() {
    ++v_counter_;
    return w_.push_v_sized_vector();
  }

 private:
  ScratchWorkspace<T>& w_;
  int xc_counter_{0};
  int xn_counter_{0};
  int xt_counter_{0};
  int v_counter_{0};
};

template <typename T>
class ProblemData {
    public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ProblemData)

  ProblemData(const T& dt, const MatrixX<T>* M, const MatrixX<T>* Jn,
              const MatrixX<T>* Jt, const VectorX<T>* tau, const VectorX<T>* v0,
              const VectorX<T>* phi0, const VectorX<T>* stiffness,
              const VectorX<T>* dissipation, const VectorX<T>* mu)
      : dt_(dt),
        M_(M),
        Jn_(Jn),
        Jt_(Jt),
        tau_(tau),
        v0_(v0),
        phi0_(phi0),
        stiffness_(stiffness),
        dissipation_(dissipation),
        mu_(mu) {
    nv_ = M_->rows();
    nc_ = phi0_->size();
  }

  int num_contacts() const { return nc_; }
  int num_velocities() const { return nv_; }

  const T& dt() const { return dt_; }
  const MatrixX<T>& M() const { return *M_;  };
  const MatrixX<T>& Jn() const { return *Jn_; };
  const MatrixX<T>& Jt() const { return *Jt_; };
  const VectorX<T>& v0() const { return *v0_;  };
  const VectorX<T>& tau() const { return *tau_; }
  const VectorX<T>& phi0() const { return *phi0_;  };
  const VectorX<T>& stiffness() const { return *stiffness_;  };
  const VectorX<T>& dissipation() const { return *dissipation_;  };
  const VectorX<T>& mu() const { return *mu_;  };

 private:        
  // sizes.
  int nc_;
  int nv_;

  // Time step
  T dt_;

  /// System's mass matrix.
  const MatrixX<T>* M_;
  const MatrixX<T>* Jn_;
  const MatrixX<T>* Jt_;

  // Right hand side.
  const VectorX<T>* tau_;
  const VectorX<T>* v0_;

  // Point pairs information.
  const VectorX<T>* phi0_;
  const VectorX<T>* stiffness_;
  const VectorX<T>* dissipation_;
  const VectorX<T>* mu_;
};

struct FBSolverParameters {
  double stiction_tolerance{1.0e-5};  // vs in [m/s].
  // stabilization parameter, tau_rigid = alpha_stab * dt.
  // With alpha_stab = 1.0 we have the position based constraint as Anitescu.
  // If alpha_stab < 0, we set v_stab = 0.
  double alpha_stab{-1.0};
  double fb_velocity_scale{1.0e-7};  // for "soft" FB.
  double m_min_factor{0.8};  // m_min = m_min_factor * mᵏ
  double delta{0.1};  // "central path" reduction factor mᵏ⁺¹ = δ⋅mᵏ.
  // Relaxation for the initial centering step (first outer iteration).
  double relaxation{0.7};
  // if max_ls_iters = 0 and limit_to_feasible_values = false, we have a fixed
  // relaxation scheme weth value "relaxation".
  int max_ls_iters = 12;

  bool limit_to_feasible_values{true};

  // Outer loop:
  int outer_loop_max_iters{400};
  double outer_loop_tolerance{1e-4};  // relative tolerance.

  // Inner loop:
  int inner_loop_max_iters{10};
  int initialization_max_iters{30};  // First outer loop to find a feasible sol.
  double inner_loop_tolerance{0.3};  // relative to the value of mᵏ.

  // Tolerance relative to the value of stiction_tolerance.
  // On convergence, the final value of m will be:
  //   m = complementary_slackness_tolerance * stiction_tolerance.
  // TODO: investigate other values, but it'd seem a value close to vs should
  // work ok.
  double complementary_slackness_tolerance{1.0};
};

struct FBSolverIterationStats {
  int total_iterations{0};
  int outer_iters{0};
  double dvn_max_norm{-1.0};
  double gpi_max_norm{-1.0};
  std::vector<int> num_inner_iters;
  int initialization_iters{0};

  // Call at the end of the inner iteration.
  void Update(double dvn_max_norm_in, double gpi_max_norm_in, int inner_iters) {
    ++outer_iters;
    total_iterations += inner_iters;
    dvn_max_norm = dvn_max_norm_in;
    gpi_max_norm = gpi_max_norm_in;
    num_inner_iters.push_back(inner_iters);
  }
};

/// The result from TamsiSolver::SolveWithGuess() used to report the
/// success or failure of the solver.
enum class FBSolverResult {
  /// Successful computation.
  kSuccess = 0,

  /// The maximum number of iterations was reached.
  kMaxIterationsReached = 1,

  /// The linear solver used within the Newton-Raphson loop failed.
  /// This might be caused by a divergent iteration that led to an invalid
  /// Jacobian matrix.
  kLinearSolverFailed = 2
};


template <typename T>
class FBSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FBSolver);

  // "cached quantities". They are all function of "state".
  struct Cache {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache);

    Cache() = default;

    // Contact velocities are all updated at once.
    bool contact_velocities_dirty{true};
    VectorX<T> vc;  // 3 * nc
    VectorX<T> vn;  // nc
    VectorX<T> vt;  // 2* nc

    // Normal constraint quantities.
    // TODO(maybe): place all together in within FBSolver::NormalConstraintsData
    bool normal_constraints_dirty{true};
    VectorX<T> Rn;  // nc
    VectorX<T> cn;  // nc
    VectorX<T> dcn_dvn;  // Diagonal.    
    VectorX<T> phi_n;    // Fische-Burmeister function for normal constraint.
    VectorX<T> gpi;      // gpi = phi_n - m_vc, residual.
    VectorX<T> dgpi_dvn;  // Diagonal matrix.
    VectorX<T> dgpi_dpi;  // Diagonal matrix.

    bool normal_constraint_jacobian_dirty{true};
    MatrixX<T> G;  // dgpi_dv;

    bool max_dissipation_constraints_dirty{true};
    VectorX<T> dlambda;
    VectorX<T> dbeta_norm;
    VectorX<T> W;
    VectorX<T> gt;

    void Resize(int nv, int nc) {
      vc.resize(3 * nc);
      vn.resize(nc);
      vt.resize(2 * nc);
      Rn.resize(nc);
      cn.resize(nc);
      dcn_dvn.resize(nc);
      gpi.resize(nc);
      phi_n.resize(nc);
      dgpi_dvn.resize(nc);
      dgpi_dpi.resize(nc);
      G.resize(nc, nv);
      dlambda.resize(nc);
      dbeta_norm.resize(nc);
      W.resize(nc);
      gt.resize(2 * nc);
    }
  };

  // The state of the iteration.
  class State {
   public:
    // N.B. We do want to copy the cache as is during assignment.
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);

    State(int nv, int nc) { Resize(nv, nc); }

    void Set(const VectorX<T>& v, const VectorX<T>& gamma, const T& m,
             const T& dt) {
      SetVelocities(v);
      SetImpulses(gamma);
      SetComplementaritySlackness(m);
      SetDt(dt);
    }

    void SetDt(const T& dt) {
      dt_ = dt;
      cache_.contact_velocities_dirty = true;
      cache_.normal_constraints_dirty = true;
      cache_.normal_constraint_jacobian_dirty = true;
    }

    void SetVelocities(const VectorX<T>& v) {
      mutable_v() = v;
    }

    void SetImpulses(const VectorX<T>& g) { mutable_gamma() = g; }

    void SetComplementaritySlackness(const T& m) {
      m_ = m;
      cache_.normal_constraints_dirty = true;
      cache_.normal_constraint_jacobian_dirty = true;
    }

    const T& dt() const { return dt_; }
    const VectorX<T>& v() const { return v_; }
    const VectorX<T>& gamma() const { return gamma_; }
    const T& m() const { return m_; }
    const Cache& cache() const { return cache_; }
    Cache& mutable_cache() const { return cache_; }
    const VectorX<T>& lambda() const { return lambda_; }    

    VectorX<T>& mutable_v() {
      cache_.contact_velocities_dirty = true;
      cache_.normal_constraints_dirty = true;
      cache_.normal_constraint_jacobian_dirty = true;
      return v_;
    }

    VectorX<T>& mutable_gamma() {
      cache_.normal_constraints_dirty = true;
      cache_.normal_constraint_jacobian_dirty = true;
      return gamma_;
    }

    VectorX<T>& mutable_lambda() {
      cache_.max_dissipation_constraints_dirty = true;
      return lambda_;
    }

    T& mutable_m() {
      // Add more granularity here, since only g, but nor phi, cn, etc depend on
      // m.
      cache_.normal_constraints_dirty = true;
      cache_.normal_constraint_jacobian_dirty = true;
      return m_;
    }

   private:
    T dt_;              // For now place here. Consider what to do.
    VectorX<T> v_;      // nv
    VectorX<T> gamma_;  // 3 * nc.
    VectorX<T> lambda_; // nc
    // The "central path" relaxation parameter for contact velocities, in [m/s]
    T m_;

    // We "cache" computations that depend on the state of the iteration.
    mutable Cache cache_;

    void Resize(int nv, int nc) {
      v_.resize(nv);
      gamma_.resize(3 * nc);
      lambda_.resize(nc);
      cache_.Resize(nv, nc);
    }
  };

  explicit FBSolver(const ProblemData<T>* data);

  // TODO: return something more meaningful.
  FBSolverResult SolveWithGuess(const VectorX<T>& v_guess);

  FBSolverResult SolveWithGuess(const State& state_guess);

  int num_contacts() const { return data_.num_contacts();  }

  int num_velocities() const { return data_.num_velocities(); }

  void set_parameters(const FBSolverParameters& p) { parameters_ = p; }

  const FBSolverIterationStats& get_stats() const { return stats_; }

  VectorX<T> get_normal_forces() const {
    VectorX<T> pi(num_contacts());
    ExtractNormal(state_.gamma(), &pi);
    return pi;
  }

  const State& get_state() const { return state_; }

 private:
  // Utilities to be used as operators.
  // Performs vc = Jc(s) * v
  void MultiplyByJc(const VectorX<T>& v, VectorX<T>* vc) const {
    // Simple operator that depends on data and not the state.
    *vc = Jc_ * v;
  }

  void MultiplyByJn(const VectorX<T>& v, VectorX<T>* vn) const {
    DRAKE_DEMAND(vn->size() == num_contacts());
    *vn = data_.Jn() * v;
  }

  void MultiplyByJt(const VectorX<T>& v, VectorX<T>* vt) const {
    DRAKE_DEMAND(vt->size() == 2 * num_contacts());
    *vt = data_.Jt() * v;
  }

  // tau_n = Jn^T * fn
  void MultiplyByJnTranspose(const VectorX<T>& fn, VectorX<T>* tau_n) const {
    DRAKE_DEMAND(fn.size() == num_contacts());                      
    DRAKE_DEMAND(tau_n->size() == num_velocities());
    *tau_n = data_.Jn().transpose() * fn;
  }

  // tau_t = Jt^T * ft
  void MultiplyByJtTranspose(const VectorX<T>& ft, VectorX<T>* tau_t) const {
    DRAKE_DEMAND(ft.size() == 2 * num_contacts());                      
    DRAKE_DEMAND(tau_t->size() == num_velocities());
    *tau_t = data_.Jt().transpose() * ft;
  }

  // Performs vn_tilde = Gn * v
  void MultiplyByGn(const State& s, const VectorX<T>& v,
                    VectorX<T>* vn_tilde) const {
    ValidateNormalConstraintsCache(s);                      
    MultiplyByJn(v, vn_tilde);
    const VectorX<T>& dgpi_dvn = s.cache().dgpi_dvn;  
    (*vn_tilde) += dgpi_dvn.asDiagonal() * (*vn_tilde);
  }

  // In this formulation Gt = Jt, i.e. vt_tilde = vt.
  void MultiplyByGt(const State&, const VectorX<T>& v,
                    VectorX<T>* vt_tilde) const {
    MultiplyByJt(v, vt_tilde);
  }

  void MultiplyByGc(const State& s, const VectorX<T>& v,
                    VectorX<T>* vc_tilde) const {
    DRAKE_DEMAND(vc_tilde->size() == 3 * num_contacts());
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& vn_tilde = access.xn_sized_vector();
    auto& vt_tilde = access.xt_sized_vector();
    MultiplyByGn(s, v, &vn_tilde);
    MultiplyByGt(s, v, &vt_tilde);
    MergeNormalAndTangent(vn_tilde, vt_tilde, vc_tilde);
  }

  // tau_n = Gn(s)^T * fn
  void MultiplyByGnTranspose(const State& s, const VectorX<T>& fn,
                             VectorX<T>* tau_n) const {
    DRAKE_DEMAND(fn.size() == num_contacts());                      
    DRAKE_DEMAND(tau_n->size() == num_velocities());
    ValidateNormalConstraintsCache(s);
    const VectorX<T>& dgpi_dvn = s.cache().dgpi_dvn;
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& fn_tilde = access.xn_sized_vector();    
    fn_tilde = dgpi_dvn.asDiagonal() * fn;
    MultiplyByJnTranspose(fn_tilde, tau_n);
  }

  // tau_t = Gt(s)^T * ft
  void MultiplyByGtTranspose(const State&, const VectorX<T>& ft,
                             VectorX<T>* tau_t) const {
    DRAKE_DEMAND(ft.size() == 2 * num_contacts());
    DRAKE_DEMAND(tau_t->size() == num_velocities());
    MultiplyByJtTranspose(ft, tau_t);
  }

  // tau_c = Gc(s)^T * fc
  void MultiplyByGcTranspose(const State& s, const VectorX<T>& fc,
                    VectorX<T>* tau_c) const {
    DRAKE_DEMAND(fc.size() == 3 * num_contacts());                      
    DRAKE_DEMAND(tau_c->size() == num_velocities());
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& fn = access.xn_sized_vector();
    auto& ft = access.xt_sized_vector();
    auto& tau = access.v_sized_vector();
    ExtractNormal(fc, &fn);
    ExtractTangent(fc, &ft);
    MultiplyByGnTranspose(s, fn, &tau);
    (*tau_c) = tau;
    MultiplyByGtTranspose(s, ft, &tau);
    (*tau_c) += tau;
  }

  // Perform a = M⁻¹⋅tau
  void MultiplyByMinv(const VectorX<T>& tau, VectorX<T>* a) const {
    DRAKE_DEMAND(tau.size() == num_velocities());
    DRAKE_DEMAND(a->size() == num_velocities());
    *a = Mi_.solve(tau);
  }

  // Compute residual of the generalized velocities:
  //   Fv = v - v_star - Mi * Gcᵀ * gamma.
  void CalcVelocitiesResidual(const State& s, VectorX<T>* Fv) const {
    const auto& v = s.v();
    const auto& gamma = s.gamma();
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& GcT_gamma = access.v_sized_vector();
    auto& Mi_GcT_gamma = access.v_sized_vector();
    MultiplyByGcTranspose(s, gamma, &GcT_gamma);  // GcT_gamma = Gc(s)ᵀ * gamma.
    MultiplyByMinv(GcT_gamma, &Mi_GcT_gamma);
    *Fv = v - v_star_ - Mi_GcT_gamma;
  }

  // Compute residual of the contact velocities:
  //  Fvc = vc_tilde - v_star_tilde - Ntilde * gamma = G * Fv
  void CalcContactVelocitiesResidual(const State& s, VectorX<T>* Fvc) const {
    DRAKE_DEMAND(Fvc->size() == 3 * num_contacts());
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& Fv = access.v_sized_vector();
    CalcVelocitiesResidual(s, &Fv);
    MultiplyByGc(s, Fv, Fvc);
  }

  // Utilities
  void ExtractNormal(const VectorX<T>& xc, VectorX<T>* xn) const {
    DRAKE_DEMAND(xc.size() == 3 * num_contacts());
    DRAKE_DEMAND(xn->size() == num_contacts());
    for (int i = 0; i < xn->size(); ++i) {
      (*xn)(i) = xc(3 * i + 2);
    }
  }

  void ExtractTangent(const VectorX<T>& xc, VectorX<T>* xt) const {
    DRAKE_DEMAND(xc.size() == 3 * num_contacts());
    DRAKE_DEMAND(xt->size() == 2 * num_contacts());
    for (int i = 0; i < num_contacts(); ++i) {
      xt->template segment<2>(2 * i) = xc.template segment<2>(3 * i);
    }
  }

  void MergeNormalAndTangent(const VectorX<T>& xn, const VectorX<T>& xt,
                                    VectorX<T>* xc) const {
    DRAKE_DEMAND(xt.size() == 2 * num_contacts());
    DRAKE_DEMAND(xc->size() == 3 * num_contacts());
    for (int i = 0; i < num_contacts(); ++i) {
      xc->template segment<2>(3 * i) = xt.template segment<2>(2 * i);
      (*xc)(3 * i + 2) = xn(i);
    }
  }

  bool CheckOuterLoopConvergenceCriteria(const VectorX<T>& v,
                                const VectorX<T>& dv, double* max_dv_norm) const;

  bool CheckInnerLoopConvergenceCriteria(const VectorX<T>& g, const T& m,
                                         double* g_max_norm) const;

  T EstimateVelocityScale(const VectorX<T>& vc,
                          const VectorX<T>& vc_star) const;

  void CalcNormalStabilizationVelocity(const T& dt, VectorX<T>* vn_stab);  

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

  const VectorX<T>& EvalVc(const State& s) {
    ValidateContactVelocitiesCache(s);
    return s.cache().vc;
  }

  const VectorX<T>& EvalVn(const State& s) {
    ValidateContactVelocitiesCache(s);
    return s.cache().vn;
  }

  const VectorX<T>& EvalVt(const State& s) {
    ValidateContactVelocitiesCache(s);
    return s.cache().vt;
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
                                    const T& m_vc, const T& dt, VectorX<T>* cn,
                                    VectorX<T>* phi,
                                    VectorX<T>* gpi, VectorX<T>* Rn = nullptr,
                                    VectorX<T>* dcn_dvn = nullptr,
                                    VectorX<T>* dgpi_dvn = nullptr,
                                    VectorX<T>* dgpi_dpi = nullptr) const;

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
      CalcNormalConstraintResidual(c.vn, pi, s.m(), s.dt(), &c.cn, &c.phi_n,
                                   &c.gpi, &c.Rn, &c.dcn_dvn, &c.dgpi_dvn,
                                   &c.dgpi_dpi);
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
                                           &c.dbeta_norm, &c.W, &c.gt);
      c.max_dissipation_constraints_dirty = false;
    }
  }

  T LimitNormalUpdate(const State& s_km, const State& s_kp,
                      int outer_iter) const;

  const ProblemData<T>& data_;
  FBSolverParameters parameters_;

  //////////////////////////////////////////////////////////////////////////////
  // ALL THIS CONST AFTER LOADING DATA.
  // The full contact Jacobian merging Jn and Jt.
  // TODO: consider that data passes Jc and then we split into Jn and Jt.
  MatrixX<T> Jc_;
  // Inverse of mass matrix in contact space.
  Eigen::LDLT<MatrixX<T>> Mi_;
  MatrixX<T> Mi_times_JcT_;
  MatrixX<T> N_;
  // Scaling factors. Multiply impulses to get velocities, in [1 / Kg].
  // One per contact, of size nc.
  VectorX<T> scaling_factor;
  VectorX<T> vn0_;
  VectorX<T> v_star_;
  VectorX<T> vn_stab_;  // Normal stabilization velocity.

  //////////////////////////////////////////////////////////////////////////////
  // SOLVER'S ITERATION STATE, GIVEN BY SOLUTION AND MULTIPLIERS.
  VectorX<T> v_;   // Generalized velocities.
  VectorX<T> pi_;  // Normal forces.

  //////////////////////////////////////////////////////////////////////////////
  // THESE QUANTITIES CHANGE DURING ITERATION.
  // They can all be computed as function of the state.
  State state_;
  mutable ScratchWorkspace<T> scratch_workspace_;

  FBSolverIterationStats stats_;
};
}
}