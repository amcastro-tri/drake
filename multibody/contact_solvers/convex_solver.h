#pragma once

#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include <Eigen/SparseCore>

#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

struct ConvexSolverParameters {
  // Over-relaxation paramter, in (0, 1]
  double relaxation{1.0};
  int max_ls_iters{15};
  double ls_factor{0.8};
  // Absolute contact velocity tolerance, m/s.
  double abs_tolerance{1.0e-8};
  // Relative contact velocity tolerance, unitless.
  double rel_tolerance{1.0e-6};
  // Maximum number of PGS iterations.
  int max_iterations{100};
  // We estimate a contact time constant as tc = alpha * time_step.
  double alpha{1.0};
  double Rt_factor{1.0};  // Rt = Rt_factor * Rn.
};

struct ConvexErrorMetrics {
  double vc_err{0.0};     // Error in the contact velocities, [m/s].
  double gamma_err{0.0};  // Scaled error in the contact impulses, [m/s].
  double id_err{0.0};     // Max-norm of scaled difference with ID, [m/s].
  double ell{0.0};        // Cost function, [Joules].
  double vs_max{-1.0};    // Maximum effective stiction tolerance, [m/s].
};

struct ConvexSolverStats {
  int iterations{0};
  std::vector<ConvexErrorMetrics> iteration_errors;
  int num_contacts{0};
  int ls_iterations{0};
};

template <typename T>
class ConvexSolver final : public ContactSolver<T> {
 public:
  // The solver pre-processes SystemDynamicsData and PointContactData before a
  // solve and stores it here.
  // Computed with PreProcessData().
  // Argument to CalcInverseDynamics().
  struct PreProcessedData {
    int nv{0};
    int nc{0};

    VectorX<T> vc_star;
    VectorX<T> vc_stab;  // Anitescu uses vc_stab = -b = (0, 0, -phi0/dt).
    VectorX<T> r;        // = vc_star - vc_stab

    // Delassus operator and related quantities:
    Eigen::SparseMatrix<T> W;
    Eigen::SparseMatrix<T> N;  // N = W + diag(R).
    // gi = trace(Wii)/3. Trace of the 3x3 diagonal block of W.
    VectorX<T> gi;
    // Diagonal preconditioning of matrix W.
    // As in [Anitescu and Tasora, 2010] we approximate each 3x3 block Bi of B
    // as Bi = I₃/gᵢ with gᵢ = tr(Wᵢᵢ)/3 the trace of the 3x3 diagonal block of
    // W. We define the effective mass of the i-th contact as
    // mᵢ = 1/gᵢ = 3/tr(Wᵢᵢ).
    VectorX<T> mi;

    // Friction and regularization:
    VectorX<T> mu;        // friction coefficients.
    VectorX<T> mu_tilde;  // mu_tilde = sqrt(Rt/Rn) * mu.
    VectorX<T> Rt;        // Regularization in the tangential direction.
    VectorX<T> Rn;        // Regularization in the normal direction.
    VectorX<T> R;         // R = (Rt, Rt, Rn)
    VectorX<T> Rinv;         // R = (1.0/Rt, 1.0/Rt, 1.0/Rn)

    void Resize(int num_velocities, int num_contacts) {
      nv = num_velocities;
      nc = num_contacts;
      const int nc3 = 3 * nc;
      W.resize(nc3, nc3);
      N.resize(nc3, nc3);
      vc_star.resize(nc3);
      gi.resize(nc);
      mi.resize(nc);
      mu.resize(nc);
      mu_tilde.resize(nc);
      Rt.resize(nc);
      Rn.resize(nc);
      R.resize(nc3);
      Rinv.resize(nc3);
      vc_stab.resize(nc3);
      r.resize(nc3);
    }
  };

  class State {
   public:
    struct Cache {
      void Resize(int nv, int nc) {
        const int nc3 = 3 * nc;
        nu.resize(nc3);
        vc.resize(nc3);
        gamma_id.resize(nc3);
      }
      // KKT multiplier to satisfy friction cone condition. ν ∈ F°.
      // Gradient of the Lagrangian leads to N⋅γ + r + ν = 0.
      // Similarly: g + R⋅γ + ν = 0, with g = vc - vc_stab.
      // HOWEVER: Analytical solution is available!
      VectorX<T> nu;        // ν = −R⋅(γ − y), with γ = Π(y).
      VectorX<T> vc;        // vc = W⋅γ + vc_star.
      VectorX<T> gamma_id;  // Analytical inverse dynamics, gamma_id = gamma(vc)
    };

    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);
    State() = default;
    void Resize(int nv, int nc) {
      nv_ = nv;
      nc_ = nc;
      gamma_.resize(3 * nc);
      cache_.Resize(nv, nc);
    }

    int num_velocities() const { return nv_; }
    int num_contacts() const { return nc_; }
    const VectorX<T>& gamma() const { return gamma_; }
    VectorX<T>& mutable_gamma() { return gamma_; }
    const Cache& cache() const { return cache_; }
    Cache& mutable_cache() const { return cache_; }

   private:
    int nv_;
    int nc_;

    // This solver's state is fully defined by the impulses gamma.
    VectorX<T> gamma_;

    mutable Cache cache_;
  };

  ConvexSolver() = default;

  virtual ~ConvexSolver() = default;

  void set_parameters(ConvexSolverParameters& parameters) {
    parameters_ = parameters;
  }

  ContactSolverStatus SolveWithGuess(const T& time_step,
                                     const SystemDynamicsData<T>& dynamics_data,
                                     const PointContactData<T>& contact_data,
                                     const VectorX<T>& v_guess,
                                     ContactSolverResults<T>* result) final;

  const ConvexSolverStats& get_iteration_stats() const { return stats_; }

  const std::vector<ConvexSolverStats>& get_stats_history() const {
    return stats_history_;
  }

  const VectorX<T>& GetContactVelocities() const { return state_.cache().vc; }
  const VectorX<T>& GetImpulses() const { return state_.gamma(); }

  // no-op at this point.
  void set_guess(const VectorX<T>& v, const VectorX<T>& gamma) {}

  PreProcessedData PreProcessData(
      const T& time_step, const SystemDynamicsData<T>& dynamics_data,
      const PointContactData<T>& contact_data) const;

  /// Given generalized velocities v, this method computes the analytical
  /// solution to the inverse dynamics problem, i.e. the impulses gamma.
  void CalcInverseDynamics(const PreProcessedData& data, const VectorX<T>& v,
                           VectorX<T>* gamma) const;

 private:
  int num_velocities() const { return state_.num_velocities(); }
  int num_contacts() const { return state_.num_contacts(); }

  // If relaxation is used, gamma_kp, vc_kp MUST be the velocities after
  // relaxation has been applied. That is, gamma_kp, vc_kp are the updates at
  // the end of the iteration.
  // Input parameter `omega` is the relaxation parameter and it is used to
  // correct for the fact that omega < 1 implies a smaller update.
  bool VerifyConvergenceCriteria(const VectorX<T>& vc, const VectorX<T>& vc_kp,
                                 const VectorX<T>& gamma,
                                 const VectorX<T>& gamma_kp,
                                 const VectorX<T>& gamma_id,
                                 double* vc_err, double* gamma_err,
                                 double* id_err) const;
  Vector3<T> ProjectImpulse(const T& mu, const T& mu_tilde, const T& Rt_over_Rn,
                            const Eigen::Ref<const Vector3<T>>& gamma) const;
  Matrix3<T> CalcProjectionMatrix(
      const T& mu, const T& mu_tilde, const T& Rt_over_Rn,
      const Eigen::Ref<const Vector3<T>>& gamma) const;
  Matrix3<T> CalcProjectionGradient(
      const T& mu, const T& mu_tilde, const T& Rt_over_Rn,
      const Eigen::Ref<const Vector3<T>>& gamma) const;
  void ProjectAllImpulses(const PreProcessedData& data,
                          const VectorX<T>& y, VectorX<T>* gamma) const;
  void FillSparseBlockDiagonalMatrix(const std::vector<Matrix3<T>>& blocks,
                                     Eigen::SparseMatrix<T>* S) const;

  int nv_, nc_;  // Problem size.
  ConvexSolverParameters parameters_;
  const SystemDynamicsData<T>* dynamics_data_{nullptr};
  const PointContactData<T>* contact_data_{nullptr};
  PreProcessedData pre_proc_data_;
  State state_;
  ConvexSolverStats stats_;
  std::vector<ConvexSolverStats> stats_history_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

extern template class ::drake::multibody::contact_solvers::internal::
    ConvexSolver<double>;
