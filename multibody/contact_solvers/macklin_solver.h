#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/system_dynamics_data.h"
#include "drake/multibody/contact_solvers/point_contact_data.h"
#include "drake/multibody/contact_solvers/scratch_workspace.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {

struct MacklinSolverParameters {
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

struct MacklinSolverIterationStats {
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

template <typename T>
class MacklinSolver : public ContactSolver<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MacklinSolver);

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
    bool normal_constraints_dirty{true};
    VectorX<T> Rn;  // nc
    VectorX<T> cn;  // nc
    VectorX<T> dcn_dvn;  // Diagonal.    
    VectorX<T> phi_n;    // Fische-Burmeister function for normal constraint.
    VectorX<T> gpi;      // gpi = phi_n - m_vc, residual.
    VectorX<T> dgpi_dvn;  // Diagonal matrix.
    VectorX<T> dgpi_dpi;  // Diagonal matrix.

    // The normal constraints are written as:
    //   gpiᵢ = ϕ(cₙᵢ(v, π), π) = 0.
    // with:
    //   cₙᵢ(v, π) = vₙᵢ − vₛᵢ + Rₙᵢ πᵢ
    // Its Jacobian is defined as:
    //   Gₙ = ∂cₙᵢ(v, π)/∂v = diag(∂cₙᵢ(v, π)/∂vₙᵢ)⋅Jₙ
    bool normal_constraint_jacobian_dirty{true};
    MatrixX<T> G;  // dgpi_dv;

    // The MDP constraints are written as:
    //   μ π vₜ + λ β = 0
    // Using the fixed-point iteration scheme in [Macklin et al. 2019] we write:
    //    vₜ + Wmdp β = 0
    bool max_dissipation_constraints_dirty{true};
    VectorX<T> dlambda;
    VectorX<T> dbeta_norm;
    VectorX<T> Wmdp;
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
      Wmdp.resize(nc);
      gt.resize(2 * nc);
      DgDvc.resize(3 * nc);
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
    VectorX<T> v_;      // nv
    VectorX<T> gamma_;  // 3 * nc.
    VectorX<T> lambda_; // nc
    // The "central path" relaxation parameter for contact velocities, in [m/s]
    T m_;

    // We "cache" computations that depend on the state of the iteration.
    mutable Cache cache_;    
  };
  
  MacklinSolver(int nv, int nc) : scratch_workspace_(nv, nc, 64) {}

  void SetSystemDynamicsData(const SystemDynamicsData<T>* data) final;

  void SetPointContactData(const PointContactData<T>* data) final;

  int num_contacts() const final { return contact_data_->num_contacts();  }

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
  void MultiplyByGc(const State& s,
                    const Eigen::Ref<const Eigen::SparseVector<T>>& v,
                    Eigen::SparseVector<T>* vc_tilde) const {
    DRAKE_DEMAND(vc_tilde->size() == 3 * num_contacts());
    get_Jc().Multiply(v, vc_tilde);                  // vc_tilde = Jc * v
    MultiplyBy_DgDvc(s, *vc_tilde, &*vc_tilde);  // vc_tilde = Gc * v
  }

  void MultiplyByGc(const State& s, const Eigen::Ref<const VectorX<T>>& v,
                    VectorX<T>* vc_tilde) const {
    DRAKE_DEMAND(vc_tilde->size() == 3 * num_contacts());
    get_Jc().Multiply(v, vc_tilde);              // vc_tilde = Jc * v
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
    get_Jc().MultiplyByTranspose(fc_tilde, &*tau_c);  // tau_c = JcT * DgDvc * fc
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
  //   Fv = v - v_star - Mi * Gcᵀ * gamma.
  void CalcVelocitiesResidual(const State& s, VectorX<T>* Fv) const {
    const auto& v = s.v();
    const auto& gamma = s.gamma();
    GrantScratchWorkspaceAccess<T> access(scratch_workspace_);
    auto& GcT_gamma = access.v_sized_vector();
    auto& Mi_GcT_gamma = access.v_sized_vector();
    MultiplyByGcTranspose(s, gamma, &GcT_gamma);  // GcT_gamma = Gc(s)ᵀ * gamma.
    get_Ainv().Multiply(GcT_gamma, &Mi_GcT_gamma);
    *Fv = v - get_v_star() - Mi_GcT_gamma;
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

#if 0
  // TODO: remove, start using the ones in contact_solver_utils.h
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
#endif  

  bool CheckOuterLoopConvergenceCriteria(const VectorX<T>& v,
                                const VectorX<T>& dv, double* max_dv_norm) const;

  bool CheckInnerLoopConvergenceCriteria(const VectorX<T>& g, const T& m,
                                         double* g_max_norm) const;

  T EstimateVelocityScale(const VectorX<T>& vc,
                          const VectorX<T>& vc_star) const;

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
      CalcNormalConstraintResidual(c.vn, pi, s.m(), get_dt(), &c.cn, &c.phi_n,
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
                                           &c.dbeta_norm, &c.Wmdp, &c.gt);
      c.max_dissipation_constraints_dirty = false;
    }
  }

  T LimitNormalUpdate(const State& s_km, const State& s_kp,
                      int outer_iter) const;

  void PreProcessData();

  // Quick accessors to problem data.
  const T& get_dt() const { return dynamics_data_->get_dt(); }
  const LinearOperator<T>& get_Jc() const {
    return contact_data_->get_Jc();
  }
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
