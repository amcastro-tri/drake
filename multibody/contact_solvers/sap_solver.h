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

struct SapSolverParameters {
  // We monitor convergence of the contact velocities.
  double abs_tolerance{1.0e-6};  // m/s
  double rel_tolerance{1.0e-6};  // Unitless.
  int max_iterations{100};       // Maximum number of Newton iterations.

  // Line-search parameters.
  double ls_alpha_max{1.5};  // Maximum line-search parameter allowed.
  // Maximum number of line-search iterations. Only used for inexact methods.
  int ls_max_iterations{40};
  // Arimijo's method parameters.
  double ls_c{1.0e-4};  // Armijo's reduction parameter.
  double ls_rho{0.8};

  // Tolerance used in impulse soft norms and soft cones. In Ns.
  // TODO(amcastro-tri): Consider this to have units of N and scale by time
  // step.
  double soft_tolerance{1.0e-7};

  // Rigid approximation contant: Rₙ = β²/(4π²)⋅Wᵢ when the contact frequency ωₙ
  // is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  double beta{1.0};

  // Dimensionless parameterization of the regularization of friction. An
  // approximation for the bound on the slip velocity is vₛ ≈ ε⋅δt⋅g.
  double sigma{1.0e-3};

  // Use supernodal algebra for the linear solver.
  bool use_supernodal_solver{true};

  // The verbosity level determines how much information to print into stdout.
  // These levels are additive. E.g.: level 2 also prints level 0 and 1 info. 0:
  // Nothing gets printed. 1: Prints problem size and error at convergence. 2:
  // Prints sparsity structure. 3: Prints stats at each iteration.
  int verbosity_level{0};
};

// This solver uses the regularized convex formulation from [Todorov 2014]. This
// class must only implement the API ContactSolver::SolveWithGuess(), please
// refer to the documentation in ContactSolver for details.
//
// - [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
//   dynamics with contacts and constraints: Theory and implementation in
//   MuJoCo. In 2014 IEEE International Conference on Robotics and Automation
//   (ICRA) (pp. 6054-6061). IEEE.
template <typename T>
class SapSolver final : public ContactSolver<T> {
 public:
  SapSolver();

  virtual ~SapSolver() = default;

  ContactSolverStatus SolveWithGuess(const T& time_step,
                                     const SystemDynamicsData<T>& dynamics_data,
                                     const PointContactData<T>& contact_data,
                                     const VectorX<T>& v_guess,
                                     ContactSolverResults<T>* result) final;

  void set_parameters(const SapSolverParameters& parameters) {
    parameters_ = parameters;
  }

 private:
  friend class SapSolverTester;

  // This is not a real cache in the CS sense (i.e. there is no tracking of
  // dependencies nor automatic validity check) but in the sense that this
  // objects stores computations that are function of the solver's state. It is
  // the responsability of the solver to keep these computations properly in
  // sync.
  class Cache {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache);

    Cache() = default;

    struct VelocitiesCache {
      void Resize(int nc) { vc.resize(3 * nc); }
      bool valid{false};
      VectorX<T> vc;
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
      void Resize(int nc) { gamma.resize(3 * nc); }
      bool valid{false};
      VectorX<T> gamma;
    };

    struct GradientsCache {
      void Resize(int nv, int nc) {
        ell_grad_v.resize(nv);
        dgamma_dy.resize(nc);
        G.resize(nc, Matrix3<T>::Zero());
        regions.resize(nc);
      }
      bool valid{false};
      VectorX<T> ell_grad_v;              // Gradient of the cost in v.
      std::vector<Matrix3<T>> dgamma_dy;  // ∂γ/∂y.
      std::vector<MatrixX<T>> G;          // G = -∂γ/∂vc.
      VectorX<int> regions;
    };

    struct CostCache {
      bool valid{false};
      T ell{0.0};   // Total primal cost.
      T ellM{0.0};  // Velocities cost.
      T ellR{0.0};  // Regularizer cost.
    };

    struct SearchDirectionCache {
      void Resize(int nv, int nc) {
        dv.resize(nv);
        dp.resize(nv);
        dvc.resize(3 * nc);
      }
      bool valid{false};
      VectorX<T> dv;     // Search direction.
      VectorX<T> dp;     // Momentum update Δp = M⋅Δv
      VectorX<T> dvc;    // Constraints velocities update.
      T d2ellM_dalpha2;  // d²ellM/dα² = Δvᵀ⋅M⋅Δv
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

    const VectorX<T>& momentum_change() const {
      return momentum_cache().momentum_change;
    }

    const VectorX<T>& gamma() const { return impulses_cache().gamma; }

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
  // function of v.
  class State {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);

    State() = default;

    State(int nv, int nc) { Resize(nv, nc); }

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
    Cache& mutable_cache() const { return cache_; }

   private:
    VectorX<T> v_;
    mutable Cache cache_;
  };

  // Structure used to store input data pre-processed for computation.
  struct PreProcessedData {
    // Constructs and empty data.
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

  // Parameters that define the projection gamma = P(y) on the friction cone ℱ
  // using the R norm.
  struct ProjectionParams {
    // Friction coefficient. It defines the friction cone ℱ.
    T mu;
    // Regularization parameters. Define the R norm.
    T Rt;  // Tangential direction.
    T Rn;  // Normal direction.
  };

  // Computes gamma = P(y) and its gradient dPdy (if requested). In addition to
  // passing y as an argument we also pass the triplet {yr, yn, that}. This
  // allow us to reuse these quantities if already computed. TODO: make static?
  Vector3<T> CalcProjection(const ProjectionParams& params,
                            const Eigen::Ref<const Vector3<T>>& y, const T& yr,
                            const T& yn,
                            const Eigen::Ref<const Vector2<T>>& that,
                            int* region, Matrix3<T>* dPdy = nullptr) const;

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

  // Compute the analytical inverse dynamics γ = γ(vc).
  // @param[in] soft_norm_tolerance tolerance used to compute the norm of the
  // tangential unprojected impulse yt, with units of Ns.
  // @param[in] vc contact velocities. On input vc.segment<3>(3*i) contains
  // velocity for the i-th contact point.
  // @param[out] gamma contact impulses. On output gamma.segment<3>(3*i)
  // contains the impulse for the i-th contact point.
  // @param[out] dgamma_dy Gradient of gamma wrt y. Not computed if nullptr.
  // @param[out] regions not computed if dgamma_dy = nullptr. Must be
  // non-nullptr if dgamma_dy is not nullptr.
  void CalcAnalyticalInverseDynamics(
      double soft_norm_tolerance, const VectorX<T>& vc, VectorX<T>* gamma,
      std::vector<Matrix3<T>>* dgamma_dy = nullptr,
      VectorX<int>* regions = nullptr) const;

  // Pack solution into ContactSolverResults.
  void PackContactResults(const PreProcessedData& data, const VectorX<T>& v,
                          const VectorX<T>& vc, const VectorX<T>& gamma,
                          ContactSolverResults<T>* result) const;

  void CalcStoppingCriteriaResidual(const State& state, T* momentum_residual,
                                    T* momentum_scale) const;

  // This is the one and only API from ContactSolver that must be implemented.
  // Refere to ContactSolverBase's documentation for details.
  ContactSolverStatus DoSolveWithGuess(const PreProcessedData& data,
                                       const VectorX<T>& v_guess,
                                       ContactSolverResults<T>* result);

  // Updates the velocity stage in `cache`.
  void UpdateVelocitiesCache(const State& state, Cache* cache) const;

  // Updates impulses stage in `cache`.
  void UpdateImpulsesCache(const State& state, Cache* cache) const;

  void UpdateCostAndGradientsCache(const State& state, Cache* cache) const;

  void UpdateCostCache(const State& state, Cache* cache) const;

  void UpdateMomentumCache(const State& state, Cache* cache) const;

  void UpdateSearchDirectionCache(const State& state, Cache* cache) const;

  // Given velocities v and search direction dv stored in `state`, this method
  // computes ℓ(α) = ℓ(v+αΔv), for a given alpha (α), and first and second
  // derivatives dℓ/dα and d²ℓ/dα².
  T CalcLineSearchCost(const State& state_v, const T& alpha,
                       State* state_alpha) const;

  // Approximation to the 1D minimization problem α = argmin ℓ(α)= ℓ(v + αΔv)
  // over α. We define ϕ(α) = ℓ₀ + α c ℓ₀', where ℓ₀ = ℓ(0) and ℓ₀' = dℓ/dα(0).
  // With this definition the Armijo condition reads ℓ(α) < ϕ(α). This
  // approximate method seeks to minimize ℓ(α) over a discrete set of values
  // given by the geometric progression αᵣ = ρʳαₘₐₓ with r an integer, 0 < ρ < 1
  // and αₘₐₓ the maximum value of α allowed. That is, the exact problem is
  // replaced by α = argmin ℓ(α)= ℓ(v + αᵣΔv) αᵣ = ρʳαₘₐₓ s.t. ℓ(α) < ϕ(α),
  // Armijo's condition.
  int PerformBackTrackingLineSearch(const State& state, T* alpha) const;

  // Solves for dv using supernodal algebra.
  void CallSupernodalSolver(const State& s, VectorX<T>* dv,
                            conex::SuperNodalSolver* solver) const;

  // Solves for dv usind dense algebra, for debugging.
  void CallDenseSolver(const State& s, VectorX<T>* dv) const;

  void PrintProblemSizes() const;
  void PrintJacobianSparsity() const;
  void PrintConvergedIterationStats(int k, const State& s) const;

  SapSolverParameters parameters_;

  // TODO: put these into a struct NonThreadSafeData.
  mutable PreProcessedData data_;
  mutable State state_;
  std::unique_ptr<conex::SuperNodalSolver> solver_;

  struct SolverStats {
    void Reset() {
      num_iters = 0;
      num_line_search_iters = 0;
      num_impulses_cache_updates = 0;
      num_gradients_cache_updates = 0;
    }
    int num_iters{0};
    int num_line_search_iters{0};

    // This also includes also includes dγ/dy updates, when gradients are
    // updated.
    int num_impulses_cache_updates{0};

    // Number of times the gradients cache is updated.
    int num_gradients_cache_updates{0};
  };
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
