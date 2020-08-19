#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/solvers/system_dynamics_data.h"
#include "drake/multibody/solvers/point_contact_data.h"
#include "drake/multibody/solvers/scratch_workspace.h"
#include "drake/multibody/solvers/contact_solver.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace solvers {

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
class FBSolver : public ContactSolver<T> {
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

    bool contact_constraints_dirty{true};
    VectorX<T> DgDvc;

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
      DgDvc.resize(3 * nc);
    }
  };

  // The state of the iteration.
  class State {
   public:
    // N.B. We do want to copy the cache as is during assignment.
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);

    State() = default;

    State(int nv, int nc) { Resize(nv, nc); }

    void Resize(int nv, int nc) {
      v_.resize(nv);
      gamma_.resize(3 * nc);
      lambda_.resize(nc);
      cache_.Resize(nv, nc);
    }

    /// Setting dt invalidates all cache entries.
    void SetDt(const T& dt) { 
      dt_ = dt; 
      cache_.contact_velocities_dirty = true;
      cache_.normal_constraints_dirty = true;
      cache_.normal_constraint_jacobian_dirty = true;
      cache_.contact_constraints_dirty = true;
      cache_.max_dissipation_constraints_dirty = true;
    }

    void Set(const VectorX<T>& v, const VectorX<T>& gamma, const T& m) {
      SetVelocities(v);
      SetImpulses(gamma);
      SetComplementaritySlackness(m);
    }

    void SetVelocities(const VectorX<T>& v) {
      mutable_v() = v;
    }

    void SetImpulses(const VectorX<T>& g) { mutable_gamma() = g; }

    void SetComplementaritySlackness(const T& m) {
      m_ = m;
      cache_.normal_constraints_dirty = true;
      cache_.normal_constraint_jacobian_dirty = true;
      cache_.contact_constraints_dirty = true;
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
      cache_.contact_constraints_dirty = true;
      return v_;
    }

    VectorX<T>& mutable_gamma() {
      cache_.normal_constraints_dirty = true;
      cache_.normal_constraint_jacobian_dirty = true;
      cache_.contact_constraints_dirty = true;
      return gamma_;
    }

    VectorX<T>& mutable_lambda() {
      cache_.max_dissipation_constraints_dirty = true;
      cache_.contact_constraints_dirty = true;
      return lambda_;
    }

    T& mutable_m() {
      // Add more granularity here, since only g, but nor phi, cn, etc depend on
      // m.
      cache_.normal_constraints_dirty = true;
      cache_.normal_constraint_jacobian_dirty = true;
      cache_.contact_constraints_dirty = true;
      return m_;
    }

   private:
    T dt_;
    VectorX<T> v_;      // nv
    VectorX<T> gamma_;  // 3 * nc.
    VectorX<T> lambda_; // nc
    // The "central path" relaxation parameter for contact velocities, in [m/s]
    T m_;

    // We "cache" computations that depend on the state of the iteration.
    mutable Cache cache_;    
  };
  
  FBSolver(int nv, int nc) : scratch_workspace_(nv, nc, 64) {}

  // TODO: this should override the parent class.
  bool supports_point_contact_data() const { return true; };

  // TODO: this should override the parent class.
  bool supports_hybrid_contact_data() const { return false; };  

  // TODO: this should override the parent class.
  void SetSystemDynamicsData(const SystemDynamicsData<T>* data) final;

  void SetPointContactData(const PointContactData<T>* data) final {
    DRAKE_DEMAND(data != nullptr);
    contact_data_ = data;
  }

  FBSolverResult SolveWithGuess(const T& dt, const VectorX<T>& v_guess);

  FBSolverResult SolveWithGuess(const State& state_guess);

  int num_contacts() const { return dynamics_data_->num_contacts();  }

  int num_velocities() const { return dynamics_data_->num_velocities(); }

  void set_parameters(const FBSolverParameters& p) { parameters_ = p; }

  const FBSolverIterationStats& get_stats() const { return stats_; }

  VectorX<T> CopyNormalForces() const {
    GrantScratchWorkspaceAccess access(scratch_workspace_);
    auto& pi = access.xn_sized_vector();
    ExtractNormal(state_.gamma(), &pi);
    return pi;
  }

  const State& get_state() const { return state_; }

 private: 
  // DgDvc is a diagonal matrix with entries DgDvc = [1, 1, DgpiDvn] for each
  // contact point.
  // yc = DgDvc * xc, is O(3*nc).
  void MultiplyBy_DgDvc(const State& s, const VectorX<T>& xc,
                        VectorX<T>* yc) const {
    DRAKE_DEMAND(3 * num_contacts() == xc.size());
    DRAKE_DEMAND(3 * num_contacts() == yc->size());
    ValidateContactConstraintsCache(s);
    const VectorX<T>& DgDvc = s.cache().DgDvc;
    *yc = DgDvc.asDiagonal() * xc;
  }

  void MultiplyBy_DgDvc(const State& s, const Eigen::SparseVector<T>& xc,
                        Eigen::SparseVector<T>* yc) const {
    DRAKE_DEMAND(3 * num_contacts() == xc.size());
    DRAKE_DEMAND(3 * num_contacts() == yc->size());
    ValidateContactConstraintsCache(s);
    const VectorX<T>& DgDvc = s.cache().DgDvc;
    *yc = DgDvc.asDiagonal() * xc;
  }

  // Gc = DgDvc * Jc
  // Then yc = Gc * v = DgDvc * Jc * v
  template <class DenseOrSparseVector>
  void MultiplyByGc(const State& s, const DenseOrSparseVector& v,
                    DenseOrSparseVector* vc_tilde) const {
    DRAKE_DEMAND(vc_tilde->size() == 3 * num_contacts());
    get_Jc().Multiply(v, vc_tilde);                  // vc_tilde = Jc * v
    MultiplyBy_DgDvc(s, *vc_tilde, &*vc_tilde);  // vc_tilde = Gc * v
  }

  // tau_c = Gc(s)^T * fc = (dgpi_dvn*Jc)^T * fc = Jc^T * dgpi_dvn * fc
  template <class DenseOrSparseVector>
  void MultiplyByGcTranspose(const State& s, const DenseOrSparseVector& fc,
                             DenseOrSparseVector* tau_c) const {
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    //auto& fc_tilde = access.xc_sized_vector();
    DenseOrSparseVector fc_tilde(3 * num_contacts());
    MultiplyBy_DgDvc(s, fc, &fc_tilde);     // fc_tilde = DgDvc * fc
    get_JcT().Multiply(fc_tilde, &*tau_c);  // tau_c = JcT * DgDvc * fc
  }

  // Define linear operator objects for Gc and GcT.
  class GcOperator : public LinearOperator<T> {
   public:
    GcOperator(const FBSolver<T>* solver, const State* state)
        : LinearOperator<T>(3 * solver->num_contacts(),
                                  solver->num_velocities()),
          solver_(solver),
          state_(state){};

    ~GcOperator() {}

    /// Performs y = Gc*x.
    virtual void Multiply(const Eigen::SparseVector<T>& x,
                          Eigen::SparseVector<T>* y) const final {
      DRAKE_DEMAND(y != nullptr);
      DRAKE_DEMAND(x.size() == this->cols());
      DRAKE_DEMAND(y->size() == this->rows());
      solver_->MultiplyByGc(*state_, x, y);
    }

    /// Performs y = Gc*x.
    virtual void Multiply(const VectorX<T>& x, VectorX<T>* y) const {
      DRAKE_DEMAND(y != nullptr);
      DRAKE_DEMAND(x.size() == this->cols());
      DRAKE_DEMAND(y->size() == this->rows());
      solver_->MultiplyByGc(*state_, x, y);
    }

   private:
    const FBSolver<T>* solver_{nullptr};
    const State* state_{nullptr};
  };

  class GcTOperator : public LinearOperator<T> {
   public:
    GcTOperator(const FBSolver<T>* solver, const State* state)
        : LinearOperator<T>(solver->num_velocities(),
                                  3 * solver->num_contacts()),
          solver_(solver),
          state_(state){};

    ~GcTOperator() {}

    /// Performs y = GcT*x.
    virtual void Multiply(const Eigen::SparseVector<T>& x,
                          Eigen::SparseVector<T>* y) const final {
      DRAKE_DEMAND(y != nullptr);
      DRAKE_DEMAND(x.size() == this->cols());
      DRAKE_DEMAND(y->size() == this->rows());
      solver_->MultiplyByGcTranspose(*state_, x, y);
    }

    /// Performs y = GcT*x.
    virtual void Multiply(const VectorX<T>& x, VectorX<T>* y) const {
      DRAKE_DEMAND(y != nullptr);
      DRAKE_DEMAND(x.size() == this->cols());
      DRAKE_DEMAND(y->size() == this->rows());
      solver_->MultiplyByGcTranspose(*state_, x, y);
    }

   private:
    const FBSolver<T>* solver_{nullptr};
    const State* state_{nullptr};
  };

#if 0
  // Utilities to be used as operators.
  // Performs vc = Jc(s) * v
  void MultiplyByJc(const VectorX<T>& v, VectorX<T>* vc) const {
    // Simple operator that depends on data and not the state.
    get_Jc().Multiply(v, &vc);
  }

  void MultiplyByJcTranspose(const VectorX<T>& fc, VectorX<T>* tau_c) const {
    get_JcT().Multiply(fc, tau_c);
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
#endif

#if 0
  void MultiplyByJn(const VectorX<T>& v, VectorX<T>* vn) const {
    DRAKE_DEMAND(vn->size() == num_contacts());
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& vc = access.xc_sized_vector();
    MultiplyByJc(v, &vc);
    ExtractNormal(vc, vn);
  }  

  void MultiplyByJt(const VectorX<T>& v, VectorX<T>* vt) const {
    DRAKE_DEMAND(vt->size() == 2 * num_contacts());
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& vc = access.xc_sized_vector();
    MultiplyByJc(v, &vc);
    ExtractTangent(vc, vt);
  }

  // tau_n = Jn^T * fn
  void MultiplyByJnTranspose(const VectorX<T>& fn, VectorX<T>* tau_n) const {
    DRAKE_DEMAND(fn.size() == num_contacts());                      
    DRAKE_DEMAND(tau_n->size() == num_velocities());
    *tau_n = data_->Jn().transpose() * fn;
  }

  // tau_t = Jt^T * ft
  void MultiplyByJtTranspose(const VectorX<T>& ft, VectorX<T>* tau_t) const {
    DRAKE_DEMAND(ft.size() == 2 * num_contacts());                      
    DRAKE_DEMAND(tau_t->size() == num_velocities());
    *tau_t = data_->Jt().transpose() * ft;
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
#endif

  // Compute residual of the generalized velocities:
  //   Fv = v - v_star - Mi * Gcᵀ * gamma.
  void CalcVelocitiesResidual(const State& s, VectorX<T>* Fv) const {
    const auto& v = s.v();
    const auto& gamma = s.gamma();
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& GcT_gamma = access.v_sized_vector();
    auto& Mi_GcT_gamma = access.v_sized_vector();
    MultiplyByGcTranspose(s, gamma, &GcT_gamma);  // GcT_gamma = Gc(s)ᵀ * gamma.
    get_Minv().Multiply(GcT_gamma, &Mi_GcT_gamma);
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

  void ValidateContactConstraintsCache(const State& s) const {
    Cache& c = s.mutable_cache();
    if (c.contact_constraints_dirty) {
      ValidateNormalConstraintsCache(s);
      const auto& DgnDvn = c.dgpi_dvn;
      GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
      auto& DgtDvt = access.xt_sized_vector();
      DgtDvt.setOnes();
      MergeNormalAndTangent(DgnDvn, DgtDvt, &c.DgDvc);
      c.contact_constraints_dirty = false;
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

  // Performs multiplication W = G * Mi * JT one column at a time.
  // G of size 3nc x nv
  // Mi of size nv x nv
  // JT of size nv x 3nc
  // 
  // Result W of size 3nc x 3nc
  void FormDelassusOperatorMatrix(const LinearOperator<T>& G,
                                  const LinearOperator<T>& Mi,
                                  const LinearOperator<T>& JT,
                                  Eigen::SparseMatrix<T>* W) const {
    DRAKE_DEMAND(G.rows() == 3 * num_contacts());
    DRAKE_DEMAND(G.cols() == num_velocities());
    DRAKE_DEMAND(Mi.rows() == num_velocities());
    DRAKE_DEMAND(Mi.cols() == num_velocities());
    DRAKE_DEMAND(JT.rows() == num_velocities());
    DRAKE_DEMAND(JT.cols() == 3 * num_contacts());
    DRAKE_DEMAND(W->rows() == 3 * num_contacts());
    DRAKE_DEMAND(W->cols() == 3 * num_contacts());

    const int nv = num_velocities();
    const int nc = num_contacts();

    Eigen::SparseVector<T> ej(3 * nc);
    // ei.makeCompressed();   // Not available for SparseVector.
    ej.coeffRef(0) = 1.0;  // Effectively allocate one non-zero entry.    


    Eigen::SparseVector<T> JTcolj(nv);    
    Eigen::SparseVector<T> MiJTcolj(nv);    
    Eigen::SparseVector<T> Wcolj(3 * nc);
    // Reserve maximum number of non-zeros.
    JTcolj.reserve(nv);
    MiJTcolj.reserve(nv);
    Wcolj.reserve(3 * nc);

    // Loop over the j-th column.
    for (int j = 0; j < W->cols(); ++j) {
      // By changing the inner index, we change what entry is the non-zero with
      // value 1.0.
      *ej.innerIndexPtr() = j;

      // Reset to nnz = 0. Memory is not freed.
      JTcolj.setZero();
      MiJTcolj.setZero();
      Wcolj.setZero();

      // Apply each operator in sequence.
      JT.Multiply(ej, &JTcolj);  // JTcolj = JT * ej
      Mi.Multiply(JTcolj, &MiJTcolj);
      G.Multiply(MiJTcolj, &Wcolj);
      W->col(j) = Wcolj;
    }
  }

  // Quick accessors to problem data.
  const LinearOperator<T>& get_Jc() const {
    return dynamics_data_->get_Jc();
  }
  const LinearOperator<T>& get_JcT() const {
    return dynamics_data_->get_JcT();
  }
  const LinearOperator<T>& get_Minv() const {
    return dynamics_data_->get_Minv();
  }
  const VectorX<T>& get_v0() const { return dynamics_data_->get_v0(); }
  const VectorX<T>& get_tau() const { return dynamics_data_->get_tau(); }
  const VectorX<T>& get_phi0() const { return contact_data_->phi0(); }
  const VectorX<T>& get_stiffness() const {
    return contact_data_->get_stiffness();
  }
  const VectorX<T>& get_dissipation() const {
    return contact_data_->get_dissipation();
  }
  const VectorX<T>& get_mu() const { return contact_data_->get_mu(); }

  const SystemDynamicsData<T>* dynamics_data_{nullptr};
  const PointContactData<T>* contact_data_{nullptr};
  FBSolverParameters parameters_;

  //////////////////////////////////////////////////////////////////////////////
  // ALL THIS CONST AFTER LOADING DATA.  
  // Inverse of mass matrix in contact space.
  Eigen::SparseMatrix<T> N_;
  // Scaling factors. Multiply impulses to get velocities, in [1 / Kg].
  // One per contact, of size nc.
  VectorX<T> scaling_factor;
  VectorX<T> vn0_;
  VectorX<T> v_star_;
  VectorX<T> vn_stab_;

  // The state of the solver's iteration.
  State state_;

  // Workspace.
  mutable ScratchWorkspace<T> scratch_workspace_;

  // Collect useful statistics.
  FBSolverIterationStats stats_;
};

}  // namespace solvers
}  // namespace multibody
}  // namespace drake