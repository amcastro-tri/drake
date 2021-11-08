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

  // Rigid approximation contant: Rₙ = α⋅Wᵢ when the contact frequency ωₙ is
  // below the limit ωₙ⋅dt ≤ 2π. That is, the period is Tₙ = α⋅dt.
  double alpha{1.0};

  // Dimensionless parameterization of the regularization of friction.
  // An approximation for the bound on the slip velocity is vₛ ≈ ε⋅δt⋅g.
  double sigma{1.0e-3};

  // Use supernodal algebra for the linear solver.
  bool use_supernodal_solver{true};

  // For debugging. Compare supernodal reconstructed Hessian with dense algebra
  // Hessian.
  bool compare_with_dense{false};

  // The verbosity level determines how much information to print into stdout.
  // These levels are additive. E.g.: level 2 also prints level 0 and 1 info.
  //  0: Nothing gets printed.
  //  1: Prints problem size and error at convergence.
  //  2: Prints sparsity structure.
  //  3: Prints stats at each iteration.
  int verbosity_level{0};
};

// Intended for debugging only. Remove.
template <typename T>
struct SolutionData {
#if 0  
  SolutionData(int nv_, int nc_) : nc(nc_) {
    const int nc3 = 3 * nc;
    vc.resize(nc3);
    gamma.resize(nc3);
    mu.resize(nc);
    R.resize(nc3);
  }
#endif
  int nc;
  VectorX<T> vc;
  VectorX<T> gamma;
  VectorX<T> mu;
  VectorX<T> R;
};

// This solver uses the regularized convex formulation from [Todorov 2014].
// This class must only implement the API ContactSolver::SolveWithGuess(),
// please refer to the documentation in ContactSolver for details.
//
// - [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
// dynamics with contacts and constraints: Theory and implementation in MuJoCo.
// In 2014 IEEE International Conference on Robotics and Automation (ICRA) (pp.
// 6054-6061). IEEE.
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
  // This is not a real cache in the CS sense (i.e. there is no tracking of
  // dependencies nor automatic validity check) but in the sense that this
  // objects stores computations that are function of the solver's state. It is
  // the responsability of the solver to keep these computations
  // properly in sync.
  struct Cache {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Cache);

    Cache() = default;

    void Resize(int nv, int nc, bool dense = true) {
      const int nc3 = 3 * nc;
      vc.resize(nc3);
      gamma.resize(nc3);
      ellR_grad_y.resize(nc3);
      ellR_hessian_y.resize(nc);
      ell_grad_v.resize(nv);
      if (dense) ell_hessian_v.resize(nv, nv);
      dv.resize(nv);
      dp.resize(nv);
      dvc.resize(nc3);
      regions.resize(nc);
      dgamma_dy.resize(nc);
      // N.B. The supernodal solver needs MatrixX instead of Matrix3.
      G.resize(nc, Matrix3<T>::Zero());
    }

    void mark_invalid() {
      valid_contact_velocity_and_impulses = false;
      valid_cost_and_gradients = false;
      valid_dense_gradients = false;
      valid_search_direction = false;
      valid_line_search_quantities = false;
    }

    // Direct algebraic funtions of velocity.
    // CalcVelocityAndImpulses() updates these entries.
    bool valid_contact_velocity_and_impulses{false};
    VectorX<T> vc;     // Contact velocities.
    VectorX<T> gamma;  // Impulses.

    bool valid_cost_and_gradients{false};
    T ell;   // The total cost.
    T ellM;  // Mass matrix cost.
    T ellR;  // The regularizer cost.
    // N.B. The supernodal solver consumes G as a vector MatrixX instead of
    // Matrix3. That is why dgamma_dy uses Matrix3 and G uses MatrixX.
    std::vector<Matrix3<T>> dgamma_dy;  // ∂γ/∂y.
    std::vector<MatrixX<T>> G;          // G = -∂γ/∂vc.
    VectorX<T> ell_grad_v;              // Gradient of the cost in v.
    VectorX<int> regions;

    // TODO: needed?
    VectorX<T> ellR_grad_y;                  // Gradient of regularizer in y.
    std::vector<Matrix3<T>> ellR_hessian_y;  // Hessian of regularizer in y.

    // TODO: only for debugging. Remove these.
    bool valid_dense_gradients{false};
    MatrixX<T> ell_hessian_v;  // Hessian in v.

    // Search directions are also a function of state. Gradients (i.e.
    // valid_cost_and_gradients) must be valid in order for the computation to
    // be correct.
    bool valid_search_direction{false};
    VectorX<T> dv;       // search direction.
    VectorX<T> dvc;      // Search direction in contact velocities.
    T condition_number;  // An estimate of the Hessian's condition number.

    // One-dimensional quantities used in line-search.
    // These depend on Δv (i.e. on valid_search_direction).
    bool valid_line_search_quantities{false};
    VectorX<T> dp;     // Δp = M⋅Δv
    T d2ellM_dalpha2;  // d2ellM_dalpha2 = Δvᵀ⋅M⋅Δv
  };

  // TODO: make into a proper class.
  struct PreProcessedData {
    void Resize(int nv_in, int nc_in) {
      nv = nv_in;
      nc = nc_in;
      const int nc3 = 3 * nc;
      R.resize(nc3);
      Rinv.resize(nc3);
      vc_stab.resize(nc3);
      Djac.resize(nv);
      p_star.resize(nv);
      Wdiag.resize(nc);
    }
    T time_step;
    const SystemDynamicsData<T>* dynamics_data{nullptr};
    const PointContactData<T>* contact_data{nullptr};
    int nv;
    int nc;
    VectorX<T> R;        // Regularization parameters, of size 3nc.
    VectorX<T> Rinv;     // Inverse of regularization parameters, of size 3nc.
    VectorX<T> vc_stab;  // Constraints stabilization velocity, see paper.
    BlockSparseMatrix<T> Jblock;  // Jacobian as block-structured matrix.
    BlockSparseMatrix<T> Mblock;  // Mass mastrix as block-structured matrix.
    std::vector<MatrixX<T>> Mt;  // Per-tree diagonal blocks of the mass matrix.
    // Jacobi pre-conditioner for the mass matrix.
    // Djac = diag(M)^(-0.5)
    VectorX<T> Djac;
    VectorX<T> p_star;
    VectorX<T> Wdiag;  // Delassus operator diagonal approximation.
  };

  // Everything in this solver is a function of the generalized velocities v.
  // State stores generalized velocities v and cached quantities that are
  // function of v.
  class State {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);

    State() = default;

    State(int nv, int nc, bool dense) { Resize(nv, nc, dense); }

    void Resize(int nv, int nc, bool dense) {
      v_.resize(nv);
      cache_.Resize(nv, nc, dense);
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

  // Parameters that define the projection gamma = P(y) on the friction cone ℱ
  // using the R norm.
  struct ProjectionParams {
    // Friction coefficient. It defines the friction cone ℱ.
    T mu;
    // Regularization parameters. Define the R norm.
    T Rt;  // Tangential direction.
    T Rn;  // Normal direction.
  };

  // Computes gamma = P(y) and its gradient dPdy (if requested).
  // In addition to passing y as an argument we also pass the triplet {yr,
  // yn, that}. This allow us to reuse these quantities if already computed.
  // TODO: make static?
  Vector3<T> CalcProjection(const ProjectionParams& params,
                            const Eigen::Ref<const Vector3<T>>& y, const T& yr,
                            const T& yn,
                            const Eigen::Ref<const Vector2<T>>& that,
                            int* region, Matrix3<T>* dPdy = nullptr) const;

  void CalcDelassusDiagonalApproximation(int nc,
                                         const std::vector<MatrixX<T>>& Mt,
                                         const BlockSparseMatrix<T>& Jblock,
                                         VectorX<T>* Wdiag) const;

  // TODO: make ti return data instead and remove member data from here.
  PreProcessedData PreProcessData(const T& time_step,
                                  const SystemDynamicsData<T>& dynamics_data,
                                  const PointContactData<T>& contact_data,
                                  double alpha,
                                  double sigma);

  // Utility to compute the "soft norm" ‖x‖ₛ defined by ‖x‖ₛ² = ‖x‖² + ε², where
  // ε = soft_tolerance.
  T SoftNorm(const Eigen::Ref<const VectorX<T>>& x,
             double soft_tolerance) const {
    using std::sqrt;
    return sqrt(x.squaredNorm() + soft_tolerance * soft_tolerance);
  }

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

  void CalcScaledMomentumAndScales(const PreProcessedData& data,
                                   const VectorX<T>& v, const VectorX<T>& gamma,
                                   T* scaled_momentum_error, T* momentum_scale,
                                   T* Ek, T* ellM, T* ellR, T* ell,
                                   VectorX<T>* v_work1, VectorX<T>* v_work2,
                                   VectorX<T>* v_work3) const;

  // This is the one and only API from ContactSolver that must be implemented.
  // Refere to ContactSolverBase's documentation for details.
  ContactSolverStatus DoSolveWithGuess(const PreProcessedData& data,
                                       const VectorX<T>& v_guess,
                                       ContactSolverResults<T>* result);

  // Update:
  //  - Contact velocities vc(v).
  //  - Contact impulses gamma(v).
  //  - Gradient ∂γ/∂y.
  void CalcVelocityAndImpulses(
      const State& state, VectorX<T>* vc, VectorX<T>* gamma,
      std::vector<Matrix3<T>>* dgamma_dy = nullptr) const;

  // Computes the cost ℓ(v) and gradients with respect to v.
  // If ell_hessian_v == nullptr we skip the expensive computation of the
  // Hessian.
  T CalcCostAndGradients(const State& state, VectorX<T>* ell_grad_v,
                         std::vector<MatrixX<T>>* G, T* ellM = nullptr,
                         T* ellR = nullptr,
                         MatrixX<T>* ell_hessian_v = nullptr) const;

  // Given velocities v and search direction dv stored in `state`, this method
  // computes ℓ(α) = ℓ(v+αΔv), for a given alpha (α), and first and second
  // derivatives dℓ/dα and d²ℓ/dα².
  T CalcLineSearchCostAndDerivatives(
      const State& state_v, const T& alpha, T* dell_dalpha, T* d2ell_dalpha2,
      State* state_alpha, T* ellM = nullptr, T* dellM_dalpha = nullptr,
      T* d2ellM_dalpha2 = nullptr, T* ellR = nullptr, T* dellR_dalpha = nullptr,
      T* d2ellR_dalpha2 = nullptr) const;

  // Approximation to the 1D minimization problem α = argmin ℓ(α)= ℓ(v + αΔv)
  // over α. We define ϕ(α) = ℓ₀ + α c ℓ₀', where ℓ₀ = ℓ(0) and ℓ₀' = dℓ/dα(0).
  // With this definition the Armijo condition reads ℓ(α) < ϕ(α).
  // This approximate method seeks to minimize ℓ(α) over a discrete set of
  // values given by the geometric progression αᵣ = ρʳαₘₐₓ with r an integer,
  // 0 < ρ < 1 and αₘₐₓ the maximum value of α allowed. That is, the exact
  // problem is replaced by
  //   α = argmin ℓ(α)= ℓ(v + αᵣΔv)
  //       αᵣ = ρʳαₘₐₓ
  //       s.t. ℓ(α) < ϕ(α), Armijo's condition.
  int CalcInexactLineSearchParameter(const State& state, T* alpha) const;

  // Solves for dv using supernodal algebra.
  void CallSupernodalSolver(const State& s, VectorX<T>* dv,
                            conex::SuperNodalSolver* solver);

  // Solves for dv usind dense algebra, for debugging.
  void CallDenseSolver(const State& s, VectorX<T>* dv);

  SapSolverParameters parameters_;

  struct Workspace {
    void Resize(int nv, int nc) {
      aux_v1.resize(nv);
      aux_v2.resize(nv);
    }
    VectorX<T> aux_v1;
    VectorX<T> aux_v2;
  };
  mutable Workspace workspace_;

  // Auxiliary state used by CalcLineSearchParameter().
  // TODO: either remove or make it an argument to CalcLineSearchParameter().
  mutable State aux_state_;
  mutable PreProcessedData data_;
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
