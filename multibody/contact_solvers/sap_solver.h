#pragma once

#include <iostream>
#include <memory>

#include <Eigen/SparseCore>

#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/supernodal_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// SAP solver parameters such as tolerances, maximum number of iterations and
// regularization parameters.
struct SapSolverParameters {
  // Stopping criteria tolerances. We monitor the optimality condition (for
  // SAP, balance of momentum), i.e. ‖∇ℓ‖ < εₐ + εᵣ max(‖p‖,‖j‖),
  // where ∇ℓ = A⋅(v−v*)−Jᵀγ is the momentum balance residual, p = A⋅v and j =
  // Jᵀ⋅γ. The norms above are defined as ‖x‖ = ‖D⋅x‖₂, where D = diag(A)*(1/2).
  double abs_tolerance{1.0e-6};  // Absolute tolerance εₐ.
  double rel_tolerance{1.0e-6};  // Relative tolerance εᵣ.
  int max_iterations{100};       // Maximum number of Newton iterations.

  // Line-search parameters.
  double ls_alpha_max{1.5};   // Maximum line search parameter allowed.
  int ls_max_iterations{40};  // Maximum number of line search iterations.
  double ls_c{1.0e-4};        // Armijo's criterion parameter.
  double ls_rho{0.8};         // Backtracking search paramter.

  // Rigid approximation contant: Rₙ = β²/(4π²)⋅Wᵢ when the contact frequency ωₙ
  // is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  double beta{1.0};

  // Dimensionless parameterization of the regularization of friction. An
  // approximation for the bound on the slip velocity is vₛ ≈ ε⋅δt⋅g.
  double sigma{1.0e-3};

  // Tolerance used in impulse soft norms. In Ns.
  double soft_tolerance{1.0e-7};

  // Use supernodal algebra for the linear solver.
  bool use_supernodal_solver{true};

  // The verbosity level determines how much information to print into stdout.
  // These levels are additive. E.g.: level 2 also prints level 0 and 1 info. 0:
  // Nothing gets printed. 1: Prints problem size and error at convergence. 2:
  // Prints sparsity structure. 3: Prints stats at each iteration.
  int verbosity_level{0};
};

// This class implements the Semi-Analytic Primal (SAP) solver described in
// [Castro et al., 2021].
//
// SAP uses the convex approximation of contact constraints by [Anitescu, 2006],
// with constraint regularization and analytical inverse dynamics introduced by
// [Todorov, 2014]. However SAP introduces a primal formulation in velocities
// instead of in impulses as done in previous work. This leads to a numerical
// scheme that warm-starts very effectively using velocities from the previous
// time step. In addition, SAP uses regularization to model physical compliance
// rather than to introduce constraint stabilization as previously done by
// [Todorov, 2014]. Please refere to [Castro et al., 2021] for details.
//
// - [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
//   Unconstrained Convex Formulation of Compliant Contact. Available online at
//   https://arxiv.org/abs/2110.10107.
// - [Anitescu, 2006] Anitescu M., 2006. Optimization-based simulation of
//   nonsmooth rigid multibody dynamics. Mathematical Programming, 105(1),
//   pp.113-143.
// - [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
//   dynamics with contacts and constraints: Theory and implementation in
//   mujoco. In 2014 IEEE International Conference on Robotics and Automation
//   (ICRA) (pp. 6054-6061). IEEE.
//
// TODO(amcastro-tri): enable AutoDiffXd support, if only for dense matrices so
// that we can test the long term performant solution.
// @tparam_nonsymbolic_scalar
template <typename T>
class SapSolver final : public ContactSolver<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapSolver);  

  // Struct used to store statistics for each solve by SolveWithGuess().
  struct SolverStats {
    // Initializes counters and time statistics to zero.
    void Reset() {
      num_iters = 0;
      num_line_search_iters = 0;
      num_impulses_cache_updates = 0;
      num_gradients_cache_updates = 0;
    }
    int num_iters{0};  // Number of Newton iterations.
    int num_line_search_iters{0};  // Total number of line search interations.

    // Number of impulse updates. This also includes also includes dγ/dy
    // updates, when gradients are updated.
    int num_impulses_cache_updates{0};

    // Number of times the gradients cache is updated.
    int num_gradients_cache_updates{0};
  };

  SapSolver() = default;
  virtual ~SapSolver() = default;

  // Solve the contact problem specified by the input data. See
  // ContactSolver::SolveWithGuess() for details. Currently, only `T = double`
  // is supported. An exception is thrown if `T != double`.
  ContactSolverStatus SolveWithGuess(const T& time_step,
                                     const SystemDynamicsData<T>& dynamics_data,
                                     const PointContactData<T>& contact_data,
                                     const VectorX<T>& v_guess,
                                     ContactSolverResults<T>* result) final;

  // New parameters will affect the next call to SolveWithGuess().
  void set_parameters(const SapSolverParameters& parameters) {
    parameters_ = parameters;
  }

  // Returns solver statistics from the last call to SolveWithGuess().
  // Statistics are reset with SolverStats::Reset() on each new call to
  // SolveWithGuess().
  const SolverStats& get_statistics() const { return stats_; }

 private:
  // This is not a real cache in the CS sense (i.e. there is no tracking of
  // dependencies nor automatic validity check) but in the sense that this class
  // stores computations that are function of the solver's state. It is the
  // responsability of the solver to keep these computations properly in sync.
  // This class does provide however some very basic level of checking. Const
  // access to specific cache entries demand that cache entries are valid (in
  // sync. with the state), E.g. Cache::impulses_cache(). Mutable access to
  // cache entries invalidates the cache entry, E.g.
  // Cache::mutable_impulses_cache(). With cache "validity" we mean a particular
  // cache entry is in sync with the velocities stored in State, E.g.
  // Cache::valid_impulses_cache(). For mathematical quantities, we attempt
  // follow the notation introduced in [Castro et al., 2021] as best as we can,
  // though with ASCII symbols and Unicode documentation.
  class Cache {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache);
    Cache() = default;

    struct VelocitiesCache {
      void Resize(int nc) { vc.resize(3 * nc); }
      bool valid{false};
      VectorX<T> vc;  // constraint velocities vc = J⋅v.
    };

    struct MomentumCache {
      void Resize(int nv) {
        p.resize(nv);
        j.resize(nv);
        momentum_change.resize(nv);
      }
      bool valid{false};
      VectorX<T> p;                // Generalized momentum p = M⋅v
      VectorX<T> j;                // Generalized impulse j = Jᵀ⋅γ
      VectorX<T> momentum_change;  // = M⋅(v−v*)
    };

    struct ImpulsesCache {
      void Resize(int nc) {
        y.resize(3 * nc);
        gamma.resize(3 * nc);
      }
      bool valid{false};
      VectorX<T> y;      // The (unprojected) impulse y = −R⁻¹⋅(vc − v̂).
      VectorX<T> gamma;  // Impulse γ = P(y), with P(y) the projection operator.
    };

    struct GradientsCache {
      void Resize(int nv, int nc) {
        ell_grad_v.resize(nv);
        dgamma_dy.resize(nc);
        G.resize(nc, Matrix3<T>::Zero());
      }
      bool valid{false};
      VectorX<T> ell_grad_v;              // Gradient of the cost in v.
      std::vector<Matrix3<T>> dgamma_dy;  // ∂γ/∂y.
      std::vector<MatrixX<T>> G;          // G = -∂γ/∂vc = dP/dy⋅R⁻¹.
    };

    struct CostCache {
      bool valid{false};
      T ell{0.0};   // Total primal cost, = ellM + ellR.
      T ellM{0.0};  // Velocities cost, = 1/2⋅(v−v*)ᵀ⋅M⋅(v−v*).
      T ellR{0.0};  // Regularizer cost, = 1/2⋅γᵀ⋅R⋅γ.
    };

    struct SearchDirectionCache {
      void Resize(int nv, int nc) {
        dv.resize(nv);
        dp.resize(nv);
        dvc.resize(3 * nc);
      }
      bool valid{false};
      VectorX<T> dv;     // Search direction.
      VectorX<T> dp;     // Momentum update Δp = M⋅Δv.
      VectorX<T> dvc;    // Constraints velocities update, Δvc=J⋅Δv.
      T d2ellM_dalpha2;  // d²ellM/dα² = Δvᵀ⋅M⋅Δv.
    };

    void Resize(int nv, int nc) {
      velocities_cache_.Resize(nc);
      momentum_cache_.Resize(nv);
      impulses_cache_.Resize(nc);
      gradients_cache_.Resize(nv, nc);
      search_direction_cache_.Resize(nv, nc);
    }

    void mark_invalid() {
      velocities_cache_.valid = false;
      momentum_cache_.valid = false;
      impulses_cache_.valid = false;
      gradients_cache_.valid = false;
      cost_cache_.valid = false;
      search_direction_cache_.valid = {false};
    }

    bool valid_velocities_cache() const { return velocities_cache_.valid; }

    const VelocitiesCache& velocities_cache() const {
      DRAKE_DEMAND(velocities_cache_.valid);
      return velocities_cache_;
    }

    VelocitiesCache& mutable_velocities_cache() {
      velocities_cache_.valid = false;
      return velocities_cache_;
    }

    bool valid_momentum_cache() const { return momentum_cache_.valid; }

    const MomentumCache& momentum_cache() const {
      DRAKE_DEMAND(momentum_cache_.valid);
      return momentum_cache_;
    }

    MomentumCache& mutable_momentum_cache() {
      momentum_cache_.valid = false;
      return momentum_cache_;
    }

    bool valid_impulses_cache() const { return impulses_cache_.valid; }

    const ImpulsesCache& impulses_cache() const {
      DRAKE_DEMAND(impulses_cache_.valid);
      return impulses_cache_;
    }

    ImpulsesCache& mutable_impulses_cache() {
      impulses_cache_.valid = false;
      return impulses_cache_;
    }

    bool valid_gradients_cache() const { return gradients_cache_.valid; }

    const GradientsCache& gradients_cache() const {
      DRAKE_DEMAND(gradients_cache_.valid);
      return gradients_cache_;
    }

    GradientsCache& mutable_gradients_cache() {
      gradients_cache_.valid = false;
      return gradients_cache_;
    }

    bool valid_cost_cache() const { return cost_cache_.valid; }

    const CostCache& cost_cache() const {
      DRAKE_DEMAND(cost_cache_.valid);
      return cost_cache_;
    }

    CostCache& mutable_cost_cache() {
      cost_cache_.valid = false;
      return cost_cache_;
    }

    bool valid_search_direction_cache() const {
      return search_direction_cache_.valid;
    }

    const SearchDirectionCache& search_direction_cache() const {
      DRAKE_DEMAND(search_direction_cache_.valid);
      return search_direction_cache_;
    }

    SearchDirectionCache& mutable_search_direction_cache() {
      search_direction_cache_.valid = false;
      return search_direction_cache_;
    }

    const VectorX<T>& vc() const { return velocities_cache().vc; }

    const VectorX<T>& gamma() const { return impulses_cache().gamma; }

    const VectorX<T>& momentum_change() const {
      return momentum_cache().momentum_change;
    }

   private:
    VelocitiesCache velocities_cache_;
    MomentumCache momentum_cache_;
    ImpulsesCache impulses_cache_;
    GradientsCache gradients_cache_;
    CostCache cost_cache_;
    SearchDirectionCache search_direction_cache_;
  };

  // Everything in this solver is a function of the generalized velocities v.
  // State stores generalized velocities v and cached quantities that are
  // functions of v.
  class State {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);

    State() = default;

    // Constructs a state for a problem with nv generalized velocities and nc
    // contact constraints.
    State(int nv, int nc) { Resize(nv, nc); }

    // Resizes the state for a problem with nv generalized velocities and nc
    // contact constraints.
    void Resize(int nv, int nc) {
      v_.resize(nv);
      cache_.Resize(nv, nc);
    }

    const VectorX<T>& v() const { return v_; }

    VectorX<T>& mutable_v() {
      // Mark all cache quantities as invalid since they all are a function of
      // velocity.
      cache_.mark_invalid();
      return v_;
    }

    const Cache& cache() const { return cache_; }

    // The state of the solver is fully described by the generalized velocities
    // v. Therefore mutating the cache does not change the state of the solver
    // and we mark this method "const". Mutable access to the cache is provided
    // to allow updating expensive quantities that are reused in multiple
    // places.
    Cache& mutable_cache() const { return cache_; }

   private:
    VectorX<T> v_;
    mutable Cache cache_;
  };

  // Structure used to store input data pre-processed for computation. For
  // mathematical quantities, we attempt follow the notation introduced in
  // [Castro et al., 2021] as best as we can, though with ASCII symbols and
  // Unicode documentation.
  struct PreProcessedData {
    // Constructs an empty data.
    PreProcessedData() = default;

    // @param nv_in Number of generalized velocities.
    // @param nc_in Number of contact constraints.
    PreProcessedData(int nv_in, int nc_in) : nv(nv_in), nc(nc_in) {
      const int nc3 = 3 * nc;
      R.resize(nc3);
      Rinv.resize(nc3);
      vhat.resize(nc3);
      mu.resize(nc);
      inv_sqrt_M.resize(nv);
      v_star.resize(nv);
      p_star.resize(nv);
      Wdiag.resize(nc);
    }

    T time_step;
    int nv;           // Numver of generalized velocities.
    int nc;           // Numver of contacts.
    VectorX<T> R;     // (Diagonal) Regularization matrix, of size 3nc.
    VectorX<T> Rinv;  // Inverse of regularization matrix, of size 3nc.
    VectorX<T> vhat;  // Constraints stabilization velocity, of size 3nc.
    VectorX<T> mu;    // Friction coefficients, of size nc.
    BlockSparseMatrix<T> Jblock;  // Jacobian as block-structured matrix.
    BlockSparseMatrix<T> Mblock;  // Mass mastrix as block-structured matrix.
    std::vector<MatrixX<T>> Mt;   // Per-tree blocks of the mass matrix.

    // Inverse of the diagonal matrix formed with the square root of the
    // diagonal entries of the mass matrix, i.e. inv_sqrt_M = diag(M)^{-1/2}.
    // This matrix is used to compute a dimensionless residual for the stopping
    // criteria.
    VectorX<T> inv_sqrt_M;

    VectorX<T> v_star;  // Free-motion generalized velocities.
    VectorX<T> p_star;  // Free motion generalized impulse, i.e. p* = M⋅v*.
    VectorX<T> Wdiag;   // Delassus operator diagonal approximation.
  };

  // Computes a diagonal approximation of the Delassus operator used to compute
  // a per constrataint diagonal scaling into Wdiag. Given an approximation Wₖₖ
  // of the block diagonal element corresponding to the k-th constraint, the
  // scaling is computed as Wdiag[k] = ‖Wₖₖ‖ᵣₘₛ = ‖Wₖₖ‖/3. See [Castro et al.
  // 2021] for details.
  void CalcDelassusDiagonalApproximation(int nc,
                                         const std::vector<MatrixX<T>>& Mt,
                                         const BlockSparseMatrix<T>& Jblock,
                                         VectorX<T>* Wdiag) const;

  // This method extracts and pre-processes input data into a format that is
  // more convenient for computation. In particular, it computes quantities
  // directly appearing in the optimization problem such as R, v̂, W, among
  // others.
  PreProcessedData PreProcessData(
      const T& time_step, const SystemDynamicsData<T>& dynamics_data,
      const PointContactData<T>& contact_data) const;

  // Computes gamma = P(y) where P(y) is the projection of y onto the friction
  // cone defined by `mu` using the norm defined by `R`. The gradient dP/dy of
  // the operator is computed if dPdy != nullptr.
  // See [Castro et al., 2021] for details on the projection operator and its
  // gradients.
  Vector3<T> CalcProjectionOntoFrictionCone(
      const T& mu, const Eigen::Ref<const Vector3<T>>& R,
      const Eigen::Ref<const Vector3<T>>& y, Matrix3<T>* dPdy = nullptr) const;

  // Computes the projection gamma = P(y) for all impulses and the gradient
  // dP/dy if dgamma_dy != nullptr.
  void ProjectImpulses(const VectorX<T>& y, VectorX<T>* gamma,
                       std::vector<Matrix3<T>>* dgamma_dy = nullptr) const;

  // Pack solution into ContactSolverResults.
  void PackContactResults(const PreProcessedData& data, const VectorX<T>& v,
                          const VectorX<T>& vc, const VectorX<T>& gamma,
                          ContactSolverResults<T>* result) const;

  // We monitor the optimality condition (for SAP, balance of momentum), i.e.
  // ‖∇ℓ‖ < εₐ + εᵣ max(‖p‖,‖j‖), where ∇ℓ = A⋅(v−v*)−Jᵀγ is the momentum
  // balance residual, p = A⋅v and j = Jᵀ⋅γ. The norms above are weighted as ‖x‖
  // = ‖D⋅x‖₂ where D = diag(A)*(1/2), such that all generalized momenta have
  // the same units (squared root of Joules).
  // This method computes momentum_residual = ‖∇ℓ‖ and momentum_scale =
  // max(‖p‖,‖j‖). See [Castro et al., 2021] for further details.
  void CalcStoppingCriteriaResidual(const State& state, T* momentum_residual,
                                    T* momentum_scale) const;

  // This is the one and only API from ContactSolver that must be implemented.
  // Refere to ContactSolverBase's documentation for details.
  ContactSolverStatus DoSolveWithGuess(const PreProcessedData& data,
                                       const VectorX<T>& v_guess,
                                       ContactSolverResults<T>* result);

  // Methods used to update cached quantities.
  void UpdateVelocitiesCache(const State& state, Cache* cache) const;
  void UpdateImpulsesCache(const State& state, Cache* cache) const;
  void UpdateCostCache(const State& state, Cache* cache) const;
  void UpdateMomentumCache(const State& state, Cache* cache) const;
  void UpdateSearchDirectionCache(const State& state, Cache* cache) const;
  void UpdateCostAndGradientsCache(const State& state, Cache* cache) const;

  // Computes the cost ℓ(α) = ℓ(vᵐ + αΔvᵐ) for line search, where vᵐ and Δvᵐ are
  // the last Newton iteration values of generalized velocities and search
  // direction, respectively. This methods uses the O(n) strategy described in
  // [Castro et al., 2021].
  T CalcLineSearchCost(const State& state_v, const T& alpha,
                       State* state_alpha) const;

  // Approximation to the 1D minimization problem α = argmin ℓ(α)= ℓ(v + αΔv)
  // over α. We define ϕ(α) = ℓ₀ + α c ℓ₀', where ℓ₀ = ℓ(0) and ℓ₀' = dℓ/dα(0).
  // With this definition the Armijo condition reads ℓ(α) < ϕ(α). This
  // approximate method seeks to minimize ℓ(α) over a discrete set of values
  // given by the geometric progression αᵣ = ρʳαₘₐₓ with r an integer, 0 < ρ < 1
  // and αₘₐₓ the maximum value of α allowed. That is, the exact problem is
  // replaced by a search over the discrete values αᵣ until Armijo's criteria is
  // satisfied. The satisfaction of Armijo's criteria allows to prove the
  // global convergence of SAP.
  T PerformBackTrackingLineSearch(const State& state,
                                  int* num_iterations) const;

  // Solves for dv using supernodal algebra.
  void CallSupernodalSolver(const State& s, VectorX<T>* dv,
                            conex::SuperNodalSolver* solver) const;

  // Solves for dv usind dense algebra, for debugging.
  void CallDenseSolver(const State& s, VectorX<T>* dv) const;

  void PrintProblemSizes() const;
  void PrintJacobianSparsity() const;
  void PrintConvergedIterationStats(int k, const State& s) const;  

  // TODO: put these into a struct NonThreadSafeData.
  SapSolverParameters parameters_;
  mutable PreProcessedData data_;
  mutable State state_;
  std::unique_ptr<conex::SuperNodalSolver> solver_;  
  mutable SolverStats stats_;
};

template <>
ContactSolverStatus SapSolver<double>::DoSolveWithGuess(
    const SapSolver<double>::PreProcessedData&, const VectorX<double>&,
    ContactSolverResults<double>*);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

extern template class ::drake::multibody::contact_solvers::internal::SapSolver<
    double>;
