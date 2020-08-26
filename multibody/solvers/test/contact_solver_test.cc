#include "drake/multibody/solvers/contact_solver.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/multibody/solvers/test/contact_solver_driver.h"
#include "drake/multibody/solvers/contact_solver_utils.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace solvers {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;

struct PgsSolverParameters {
  // Over-relaxation paramter, in (0, 1]
  double relaxation{0.6};
  // Absolute contact velocity tolerance, m/s.
  double abs_tolerance{1.0e-6};
  // Relative contact velocity tolerance, unitless.
  double rel_tolerance{1.0e-4};
  // Maximum number of PGS iterations.
  int max_iterations{100};
};

struct PgsSolverStats {
  int iterations{0};  // Number of PGS iterations.
  double vc_err{0.0};  // Error in the contact velocities, [m/s].
  double gamma_err{0.0};  // Error in the contact impulses, [Ns].
};

template <typename T>
class PgsSolver final : public ContactSolver<T> {
 public:
  class State {    
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);
    State() = default;
    void Resize(int nv, int nc) {
      v_.resize(nv);
      gamma_.resize(3 * nc);
    }
    const VectorX<T>& v() const { return v_; }
    VectorX<T>& mutable_v() { return v_; }
    const VectorX<T>& gamma() const { return gamma_; }
    VectorX<T>& mutable_gamma() { return gamma_; }

   private:
    VectorX<T> v_;
    VectorX<T> gamma_;
  };

  PgsSolver() = default;

  virtual ~PgsSolver() = default;

  void SetSystemDynamicsData(const SystemDynamicsData<T>* data) final {
    DRAKE_DEMAND(data != nullptr);
    dynamics_data_ = data;
  }

  virtual void SetPointContactData(const PointContactData<T>* data) final {
    DRAKE_DEMAND(data != nullptr);
    contact_data_ = data;
  }

  int num_contacts() const final { return contact_data_->num_contacts(); };
  int num_velocities() const final { return dynamics_data_->num_velocities(); }

  void set_parameters(PgsSolverParameters& parameters) {
    parameters_ = parameters;
  }

  ContactSolverResult SolveWithGuess(double dt,
                                     const VectorX<T>& v_guess) final {
    PreProcessData(dt);
    // Aliases to pre-processed (const) data.
    const auto& v_star = pre_proc_data_.v_star;
    const auto& vc_star = pre_proc_data_.vc_star;
    const auto& Dinv = pre_proc_data_.Dinv;

    // Aliases so solver's state.
    auto& v = state_.mutable_v();
    auto& gamma = state_.mutable_gamma();

    // Aliases to parameters.
    const int max_iters = parameters_.max_iterations;
    const double omega = parameters_.relaxation;

    // Set initial guess.
    v = v_guess;
    gamma.setZero();  // we don't know any better.

    // Below we use index k to denote the iteration. Hereinafter we'll adopt the
    // convention of appending a trailing _kp ("k plus") to refer to the next
    // iterate k+1.
    State state_kp(state_);  // Next iteration, k+1, state.
    VectorX<T>& v_kp = state_kp.mutable_v();
    VectorX<T>& gamma_kp = state_kp.mutable_gamma();    

    // State dependent quantities.
    vc_ = vc_star;  // Contact velocity at state_, intialized to when gamma = 0.
    VectorX<T> vc_kp(3 * num_contacts());  // Contact velocity at state_kp.
    for (int k = 0; k < max_iters; ++k) {
      gamma_kp = gamma - omega * Dinv.asDiagonal() * vc_;
      ProjectAllImpulses(vc_, &gamma_kp);
      // Update generalized velocities; v = v* + M⁻¹⋅Jᵀ⋅γ.
      get_Jc().MultiplyByTranspose(gamma_kp, &tau_c_);  // tau_c = Jᵀ⋅γ
      get_Minv().Multiply(tau_c_, &v_kp);  // v_kp = M⁻¹⋅Jᵀ⋅γ
      v_kp += v_star;  // v_kp = v* + M⁻¹⋅Jᵀ⋅γ
      // Update contact velocities; vc = J⋅v.
      get_Jc().Multiply(v_kp, &vc_kp);

      // Verify convergence and update stats.
      const bool converged = VerifyConvergenceCriteria(
          vc_, vc_kp, gamma, gamma_kp, &stats_.vc_err, &stats_.gamma_err);
      stats_.iterations++;

      // Update state for the next iteration.    
      state_ = state_kp;
      vc_ = vc_kp;
      if (converged) {
        return ContactSolverResult::kSuccess;
      }
    }
    return ContactSolverResult::kFailure;
  }

  const PgsSolverStats& get_iteration_stats() const { return stats_; }

  const VectorX<T>& GetImpulses() const final { return state_.gamma(); }
  const VectorX<T>& GetVelocities() const final { return state_.v(); }
  const VectorX<T>& GetGeneralizedContactImpulses() const final {
    return tau_c_;
  }
  const VectorX<T>& GetContactVelocities() const final { return vc_; }

 private:
  // All this data must remain const after the call to PreProcessData().
  struct PreProcessedData {
    Eigen::SparseMatrix<T> N;
    VectorX<T> vc_star;
    VectorX<T> v_star;
    // Norm of the 3x3 block diagonal block of matrix N, of size nc.
    VectorX<T> Nii_norm;
    // Approximation to the inverse of the diagonal of N, of size nc.
    VectorX<T> Dinv;
    void Resize(int nv, int nc) {
      N.resize(3 * nc, 3 * nc);
      vc_star.resize(3 * nc);
      v_star.resize(nv);
      Nii_norm.resize(nc);
      Dinv.resize(3 * nc);
    }
  };

  // Quick accessors to problem data.
  const LinearOperator<T>& get_Jc() const { return contact_data_->get_Jc(); }
  const LinearOperator<T>& get_Minv() const {
    return dynamics_data_->get_Minv();
  }
  const VectorX<T>& get_v0() const { return dynamics_data_->get_v0(); }
  const VectorX<T>& get_tau() const { return dynamics_data_->get_tau(); }
  const VectorX<T>& get_phi0() const { return contact_data_->get_phi0(); }
  const VectorX<T>& get_mu() const { return contact_data_->get_mu(); }


  void PreProcessData(double dt) {
    const int nc = num_contacts();
    const int nv = num_velocities();
    state_.Resize(nv, nc);
    pre_proc_data_.Resize(nv, nc);
    tau_c_.resize(nv);
    vc_.resize(3 * nc);

    // Generalized velocities when contact forces are zero.
    auto& v_star = pre_proc_data_.v_star;
    get_Minv().Multiply(get_tau(), &v_star);  // v_star = M⁻¹⋅tau    
    v_star = get_v0() + dt * v_star;          // v_star = v₀ + dt⋅M⁻¹⋅τ

    if (nc != 0) {
      // Contact velocities when contact forces are zero.
      auto& vc_star = pre_proc_data_.vc_star;  
      get_Jc().Multiply(v_star, &vc_star);

      auto& N = pre_proc_data_.N;
      this->FormDelassusOperatorMatrix(get_Jc(), get_Minv(), get_Jc(), &N);
      PRINT_VARn(N);

      // Compute scaling factors, one per contact.
      auto& Nii_norm = pre_proc_data_.Nii_norm;
      auto& Dinv = pre_proc_data_.Dinv;
      for (int i = 0; i < nc; ++i) {
        // 3x3 diagonal block. It might be singular, but definitely non-zero.
        // That's why we use an rms norm.
        const auto& Nii = N.block(3 * i, 3 * i, 3, 3);
        Nii_norm(i) = Nii.norm() / 3;  // 3 = sqrt(9).
        Dinv.template segment<3>(3 * i).setConstant(1.0 / Nii_norm(i));
      }
      PRINT_VAR(Nii_norm.transpose());

      PRINT_VAR(Dinv.transpose());
    }
  }

  bool VerifyConvergenceCriteria(const VectorX<T>& vc, const VectorX<T>& vc_kp,
                                 const VectorX<T>& gamma,
                                 const VectorX<T>& gamma_kp, T* vc_err,
                                 T* gamma_err) const {
    using std::max;
    const auto& Nii_norm = pre_proc_data_.Nii_norm;
    bool converged = true;
    *vc_err = 0;
    *gamma_err = 0;
    for (int ic = 0; ic < num_contacts(); ++ic) {
      auto within_error_bounds = [&p = parameters_](const T& error,
                                                     const T& scale) {
        const T bounds = p.abs_tolerance + p.rel_tolerance * scale;
        return error < bounds;
      };
      // Check velocity convergence.
      const auto vci = vc.template segment<3>(3 * ic);
      const auto vci_kp = vc_kp.template segment<3>(3 * ic);
      const T vc_norm = vci.norm();
      const T vci_err = (vci_kp - vci).norm();
      *vc_err = max(*vc_err, vci_err);
      if (!within_error_bounds(vci_err, vc_norm)) {
        converged = false;
      }

      // Check impulse convergence. Scaled to velocity so that its convergence
      // metric is compatible with that of contact velocity.
      const auto gi = gamma.template segment<3>(3 * ic);
      const auto gi_kp = gamma_kp.template segment<3>(3 * ic);
      const T g_norm = gi.norm() / Nii_norm(ic);
      T g_err = (gi_kp - gi).norm();
      *gamma_err = max(*gamma_err, g_err);
      g_err /= Nii_norm(ic);
      if (!within_error_bounds(g_err, g_norm)) {
        converged = false;
      }
    }
    return converged;
  }

  void ProjectAllImpulses(const VectorX<T>& vc, VectorX<T>* gamma_inout) const {
    VectorX<T>& gamma = *gamma_inout;
    const auto& mu = get_mu();
    for (int ic = 0; ic < num_contacts(); ++ic) {
      auto vci = vc.template segment<3>(3 * ic);
      auto gi = gamma.template segment<3>(3 * ic);
      gi = ProjectImpulse(vci, gi, mu(ic));
    }
  }

  Vector3<T> ProjectImpulse(const Eigen::Ref<const Vector3<T>>& vc,
                            const Eigen::Ref<const Vector3<T>>& gamma,
                            const T& mu) const {
    const T& pi = gamma(2);                    // Normal component.
    if (pi <= 0.0) return Vector3<T>::Zero();  // No contact.

    const auto beta = gamma.template head<2>();  // Tangential component.
    if (beta.norm() <= mu * pi) return gamma;    // Inside the cone.

    // Non-zero impulse lies outside the cone. We'll project it.
    using std::sqrt;
    // We use the absolute tolerance as a velocity scale to use in a velocity
    // soft norm.
    const double v_eps = parameters_.abs_tolerance;
    const double v_eps2 = v_eps * v_eps;
    // Alias to tangential velocity.
    const auto vt = vc.template head<2>();
    // Compute a direction.
    const T vt_soft_norm = sqrt(vt.squaredNorm() + v_eps2);
    const Vector2<T> that = vt / vt_soft_norm;
    // Project. Principle of maximum dissipation.
    const Vector2<T> projected_beta = -mu * pi * that;
    return Vector3<T>(projected_beta(0), projected_beta(1), pi);
  }

  PgsSolverParameters parameters_;
  const SystemDynamicsData<T>* dynamics_data_{nullptr};
  const PointContactData<T>* contact_data_{nullptr};
  PreProcessedData pre_proc_data_;
  State state_;
  PgsSolverStats stats_;
  // Workspace (many of these could live in the state as "cached" data.)
  VectorX<T> tau_c_;  // Generalized contact impulses.
  VectorX<T> vc_;  // Contact velocities.  
};

GTEST_TEST(ContactSolver, PizzaSaver) {
  const std::string model_file = "drake/multibody/solvers/test/pizza_saver.sdf";
  const double dt = 1.0e-3;
  test::ContactSolverDriver driver;
  driver.BuildModel(dt, model_file);  
  const auto& plant = driver.plant();
  const int nv = plant.num_velocities();
  const int nq = plant.num_positions();  
  ASSERT_EQ(nq, 7);
  ASSERT_EQ(nv, 6);

  const auto& body = plant.GetBodyByName("body");
  //driver.SetPointContactParameters(sphere, kStiffness, kDamping);
  auto& context = driver.mutable_plant_context();

  // For this test we want the ground to have the same friction as "body".
  const auto per_geometry_mu = driver.GetDynamicFrictionCoefficients(body);
  ASSERT_EQ(per_geometry_mu.size(), 3u);
  // Verify they are all equal.
  EXPECT_EQ(per_geometry_mu[0], per_geometry_mu[1]);
  EXPECT_EQ(per_geometry_mu[1], per_geometry_mu[2]);
  PRINT_VAR(per_geometry_mu[2]);
  driver.SetDynamicFrictionCoefficient(plant.world_body(), per_geometry_mu[0]);

  std::vector<ExternallyAppliedSpatialForce<double>> forces(1);
  forces[0].body_index = body.index();
  forces[0].p_BoBq_B = Vector3d::Zero();
  forces[0].F_Bq_W =
      SpatialForce<double>(Vector3d(0.0, 0.0, 3.0), Vector3d(0.0, 0.0, 0.0));
  plant.get_applied_spatial_force_input_port().FixValue(&context, forces);

  const double phi0 = -1.0e-3;  // Initial penetration into the ground, [m].
  plant.SetFreeBodyPose(&context, body,
                        RigidTransformd(Vector3d(0, 0, phi0)));

  // Visualize initial condition.
  driver.Publish();  // for viz.

  PgsSolver<double>& solver = driver.mutable_plant().set_contact_solver(
      std::make_unique<PgsSolver<double>>());

  // Specificaction of solver paramters happens here.
  PgsSolverParameters parameters;
  solver.set_parameters(parameters);

  auto params = driver.GetPointContactComplianceParameters(body);
  for (auto& p : params) {
    PRINT_VAR(p.first);
    PRINT_VAR(p.second);
  }

  driver.AdvanceNumSteps(1);

  PRINT_VAR(solver.GetImpulses().transpose());
  PRINT_VAR(solver.GetVelocities().transpose());
  PRINT_VAR(solver.GetContactVelocities().transpose());
  PRINT_VAR(solver.GetGeneralizedContactImpulses().transpose());

  auto& stats = solver.get_iteration_stats();
  PRINT_VAR(stats.iterations);
  PRINT_VAR(stats.vc_err);
  PRINT_VAR(stats.gamma_err);
  

  //const VectorX<double> v0 = VectorX<double>::Zero(nv);
  //driver.AdvanceOneStep(v0, dt);
  //driver.Publish();  // for viz.
}

#if 0

// This tests the solver when we apply a moment Mz about COM to the pizza saver.
// If Mz < mu * m * g * R, the saver should be in stiction (that is, the sliding
// velocity should be smaller than the regularization parameter). Otherwise the
// saver will start sliding. For this setup the transition occurs at
// M_transition = mu * m * g * R = 5.0
TEST_F(PizzaSaver, SmallAppliedMoment) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.5;

  // Some arbitrary orientation. This particular case has symmetry of
  // revolution (meaning the result is independent of angle theta).
  const double theta = M_PI / 5;

  // External forcing.
  const double Mz = 3.0;  // M_transition = 5.0
  const Vector3<double> tau(0.0, 0.0, Mz);

  // Initial velocity.
  const Vector3<double> v0 = Vector3<double>::Zero();

  SetProblem(v0, tau, mu, theta, dt);

  TamsiSolverParameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  parameters.relative_tolerance = 1.0e-4;
  solver_.set_solver_parameters(parameters);

  TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, TamsiSolverResult::kSuccess);

  VectorX<double> tau_f = solver_.get_generalized_friction_forces();

  const auto& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      // Dimensionless relative (to the stiction tolerance) tolerance.
      solver_.get_solver_parameters().relative_tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  // For this problem we expect the x and y components of the forces due to
  // friction to be zero.
  EXPECT_NEAR(tau_f(0), 0.0, kTolerance);
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

  // The moment due to friction should balance the applied Mz. However, it will
  // take several time steps until tau_f balances Mz (eventually it will).
  // Therefore, here we just sanity check that Mz is at least relatively close
  // (to the value of Mz) to tau_f. In other words, with only a single time
  // step, we are still accelerating towards the final steady state slip
  // introduced by having a finite stiction tolerance.
  EXPECT_NEAR(tau_f(2), -Mz, 5.0e-4);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // The problem has symmetry of revolution. Thus, for any rotation theta,
  // the three tangential velocities should have the same magnitude.
  const double v_slipA = vt.segment<2>(0).norm();
  const double v_slipB = vt.segment<2>(2).norm();
  const double v_slipC = vt.segment<2>(4).norm();
  EXPECT_NEAR(v_slipA, v_slipB, kTolerance);
  EXPECT_NEAR(v_slipA, v_slipC, kTolerance);
  EXPECT_NEAR(v_slipC, v_slipB, kTolerance);

  // For this case where Mz < M_transition, we expect stiction (slip velocities
  // are smaller than the regularization parameter).
  EXPECT_LT(v_slipA, parameters.stiction_tolerance);
  EXPECT_LT(v_slipB, parameters.stiction_tolerance);
  EXPECT_LT(v_slipC, parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities to be zero.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);

  // Compute the Newton-Raphson Jacobian of the residual J = ∇ᵥR using the
  // solver's internal implementation.
  MatrixX<double> J =
      TamsiSolverTester::CalcJacobian(solver_, v, dt);

  // Compute the same Newton-Raphson Jacobian of the residual J = ∇ᵥR but with
  // a completely separate implementation using automatic differentiation.
  const double v_stiction = parameters.stiction_tolerance;
  const double epsilon_v = v_stiction * parameters.relative_tolerance;
  MatrixX<double> J_expected = test::CalcOneWayCoupledJacobianWithAutoDiff(
      M_, Jn_, Jt_, p_star_, mu_, fn_, dt, v_stiction, epsilon_v, v);

  // We use a tolerance scaled by the norm and size of the matrix.
  const double J_tolerance =
      J_expected.rows() * J_expected.norm() *
          std::numeric_limits<double>::epsilon();

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));
}

// Exactly the same problem as in PizzaSaver::SmallAppliedMoment but with an
// applied moment Mz = 6.0 > M_transition = 5.0. In this case the pizza saver
// transitions to sliding with a net moment of Mz - M_transition during a
// period (time stepping interval) dt. Therefore we expect a change of angular
// velocity given by Δω = dt (Mz - Mtransition) / I.
TEST_F(PizzaSaver, LargeAppliedMoment) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.5;  // Friction coefficient.

  // Some arbitrary orientation. This particular case has symmetry of
  // revolution.
  const double theta = M_PI / 5;

  // External forcing.
  const double M_transition = 5.0;
  const double Mz = 6.0;
  const Vector3<double> tau(0.0, 0.0, Mz);

  // Initial velocity.
  const Vector3<double> v0 = Vector3<double>::Zero();

  SetProblem(v0, tau, mu, theta, dt);

  TamsiSolverParameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  parameters.relative_tolerance = 1.0e-4;
  solver_.set_solver_parameters(parameters);

  TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, TamsiSolverResult::kSuccess);

  VectorX<double> tau_f = solver_.get_generalized_friction_forces();

  const auto& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      // Dimensionless relative (to the stiction tolerance) tolerance.
      solver_.get_solver_parameters().relative_tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  // For this problem we expect the x and y components of the forces due to
  // friction to be zero.
  EXPECT_NEAR(tau_f(0), 0.0, kTolerance);
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);
  // Since we are sliding, the total moment should match M_transition.
  // The difference with Mz is what makes the saver to start accelerating.
  EXPECT_NEAR(tau_f(2), -M_transition, 1.0e-13);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // The problem has symmetry of revolution. Thus, for any rotation theta,
  // the three tangential velocities should have the same magnitude.
  const double v_slipA = vt.segment<2>(0).norm();
  const double v_slipB = vt.segment<2>(2).norm();
  const double v_slipC = vt.segment<2>(4).norm();
  EXPECT_NEAR(v_slipA, v_slipB, kTolerance);
  EXPECT_NEAR(v_slipA, v_slipC, kTolerance);
  EXPECT_NEAR(v_slipC, v_slipB, kTolerance);

  // For this case where Mz > M_transition, we expect sliding, so that expected
  // velocities are larger than the stiction tolerance.
  EXPECT_GT(v_slipA, parameters.stiction_tolerance);
  EXPECT_GT(v_slipB, parameters.stiction_tolerance);
  EXPECT_GT(v_slipC, parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities for the COM to be
  // zero. Still, there is slip at points A, B, C.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);

  const double omega = dt * (Mz - 5.0) / I_;
  EXPECT_NEAR(v(2), omega, kTolerance);

  // Slip velocities should only be due to rotation.
  EXPECT_NEAR(v_slipA, R_ * omega, kTolerance);
  EXPECT_NEAR(v_slipB, R_ * omega, kTolerance);
  EXPECT_NEAR(v_slipC, R_ * omega, kTolerance);

  // Compute the Newton-Raphson Jacobian of the residual J = ∇ᵥR using the
  // solver's internal implementation.
  MatrixX<double> J =
      TamsiSolverTester::CalcJacobian(solver_, v, dt);

  // Compute the same Newton-Raphson Jacobian of the residual J = ∇ᵥR but with
  // a completely separate implementation using automatic differentiation.
  const double v_stiction = parameters.stiction_tolerance;
  const double epsilon_v = v_stiction * parameters.relative_tolerance;
  MatrixX<double> J_expected = test::CalcOneWayCoupledJacobianWithAutoDiff(
      M_, Jn_, Jt_, p_star_, mu_, fn_, dt, v_stiction, epsilon_v, v);

  // We use a tolerance scaled by the norm and size of the matrix.
  const double J_tolerance = J_expected.rows() * J_expected.norm() *
      std::numeric_limits<double>::epsilon();

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));
}

// Verify the solver behaves correctly when the problem data contains no
// contact points.
TEST_F(PizzaSaver, NoContact) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  const double dt = 1.0e-3;  // time step in seconds.

  // External forcing.
  const double Mz = 6.0;
  const Vector3<double> tau(0.0, 0.0, Mz);

  // Initial velocity.
  const Vector3<double> v0 = Vector3<double>::Zero();

  SetNoContactProblem(v0, tau, dt);

  TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, TamsiSolverResult::kSuccess);

  EXPECT_EQ(solver_.get_generalized_friction_forces(), Vector3<double>::Zero());

  const auto& stats = solver_.get_iteration_statistics();
  EXPECT_EQ(stats.vt_residual(), 0);
  EXPECT_EQ(stats.num_iterations, 1);

  // Verify solution.
  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities to be zero.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);
  // Expected angular velocity change about z due to the applied moment Mz.
  const double omega = dt * Mz / I_;
  EXPECT_NEAR(v(2), omega, kTolerance);

  // No contact.
  EXPECT_EQ(solver_.get_tangential_velocities().size(), 0);
}
#endif

}  // namespace
}  // namespace solvers
}  // namespace multibody
}  // namespace drake
