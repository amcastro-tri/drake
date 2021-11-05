#include "drake/multibody/contact_solvers/sap_solver.h"

#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/multibody/contact_solvers/block_sparse_linear_operator.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/system_dynamics_data.h"
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Top view of the pizza saver:

  ^ y               C
  |                 ◯
  |                /\
  ----> x         /  \
               b /    \ a
                /      \
               /        \
              ◯----------◯
              A     c    B

It is modeled as an equilateral triangle with a contact point at each of the
legs. The total mass of the pizza saver is m and its rotational inertia about
the triangle's barycenter is I.
If h is the height of the triangle from any of its sides, the distance from
any point to the triangle's center is 2h/3. The height h relates to the length
a of a side by h = 3/2/sqrt(3) a.
The generalized positions vector for this case is q = [x, y, theta], with
theta = 0 for the triangle in the configuration shown in the schematic. */
class PizzaSaverProblem {
 public:
  static constexpr int num_velocities = 4;
  static constexpr int num_contacts = 3;

  struct ProblemData {
    ProblemData(int nv, int nc) {
      const int nc3 = 3 * nc;
      M.resize(nv, nv);
      v_star.resize(nv);
      phi0.resize(nc);
      vc0.resize(nc3);
      stiffness.resize(nc);
      dissipation.resize(nc);
      mu.resize(nc);
    }

    // Mass matrix in different formats:
    MatrixXd M;
    BlockSparseMatrix<double> Mblock;
    std::unique_ptr<BlockSparseLinearOperator<double>> Mop;

    // Jacobian in different formats:
    MatrixXd J;
    BlockSparseMatrix<double> Jblock;
    std::unique_ptr<BlockSparseLinearOperator<double>> Jop;

    // Free motion velocities.
    VectorXd v_star;

    // Contact data.
    VectorXd phi0;
    VectorXd vc0;
    VectorXd stiffness;
    VectorXd dissipation;
    VectorXd mu;

    // Contact solver data.
    std::unique_ptr<SystemDynamicsData<double>> dynamics_data;
    std::unique_ptr<PointContactData<double>> contact_data;
  };

  PizzaSaverProblem(double dt, double mass, double radius, double mu, double k,
                    double taud)
      : time_step_(dt),
        m_(mass),
        R_(radius),
        mu_(mu),
        stiffness_(k),
        taud_(taud) {
    // The radius of the circumscribed circle R is the distance from each
    // contact point to the triangle's center.
    // We model the pizza saver as three point masses m/3 at each contact
    // point and thus the moment of inertia is I = 3 * (m/3 R²) = m R²:
    I_ = m_ * R_ * R_;
  }

  // Pizza saver model with default mass m = 1 Kg and radius = 1.0 m.
  PizzaSaverProblem(double dt, double mu, double k, double taud)
      : PizzaSaverProblem(dt, 1.0, 1.0, mu, k, taud) {}

  double time_step() const { return time_step_; }

  // Mass of each point mass, Kg. Total mass is three times mass().
  double mass() const { return m_; }

  double radius() const { return R_; }

  double rotational_inertia() const { return I_; }

  // Acceleration of gravity, m/s².
  double g() const { return g_; }

  void CalcMassMatrix(MatrixXd* M) const {
    M->resize(num_velocities, num_velocities);
    // clang-format off
    *M << m_,  0,  0,  0,
           0, m_,  0,  0,
           0,  0, m_,  0,
           0,  0,  0, I_;
    // clang-format on
  }

  void CalcContactJacobian(double theta, MatrixXd* Jc) const {
    Jc->resize(3 * num_contacts, num_velocities);

    const double c = cos(theta);
    const double s = sin(theta);

    // 3D rotation matrix of the body frame B in the world frame W.
    // clang-format off
    Matrix3d R_WB;
    R_WB << c, s, 0,
           -s, c, 0,
            0, 0, 1;
    // clang-format on

    // Position of each contact point in the body frame B.
    const Vector3d p_BoA(-sqrt(3) / 2.0, -0.5, 0.0);
    const Vector3d p_BoB(sqrt(3) / 2.0, -0.5, 0.0);
    const Vector3d p_BoC(0.0, 1.0, 0.0);

    // Position of each contact point in the world frame W.
    const Vector3d p_BoA_W = R_WB * p_BoA;
    const Vector3d p_BoB_W = R_WB * p_BoB;
    const Vector3d p_BoC_W = R_WB * p_BoC;

    // clang-format off
    // Point A
    Jc->block(0, 0, 3, num_velocities) << 1, 0, 0, -p_BoA_W.y(), 
                                          0, 1, 0,  p_BoA_W.x(),
                                          0, 0, 1, 0;

    // Point B
    Jc->block(3, 0, 3, num_velocities) << 1, 0, 0, -p_BoB_W.y(), 
                                          0, 1, 0, p_BoB_W.x(),
                                          0, 0, 1, 0;

    // Point C
    Jc->block(6, 0, 3, num_velocities) << 1, 0, 0, -p_BoC_W.y(), 
                                          0, 1, 0, p_BoC_W.x(),
                                          0, 0, 1, 0;
    // clang-format on
  }

  std::unique_ptr<ProblemData> MakeProblemData(const VectorXd& q0,
                                               const VectorXd& v0,
                                               const VectorXd& tau) const {
    // Set system dynamics data:
    auto data = std::make_unique<ProblemData>(num_velocities, num_contacts);
    CalcMassMatrix(&data->M);
    {
      // A single block size.
      BlockSparseMatrixBuilder<double> builder(1, 1, 1);
      builder.PushBlock(0, 0, data->M);
      data->Mblock = builder.Build();
    }
    data->Mop =
        std::make_unique<BlockSparseLinearOperator<double>>("M", &data->Mblock);

    data->v_star.resize(num_velocities);
    data->v_star = v0 + time_step_ * data->M.ldlt().solve(tau);

    data->dynamics_data = std::make_unique<SystemDynamicsData<double>>(
        data->Mop.get(), nullptr, &data->v_star);

    // Set contact data:
    // Some arbitrary orientation. This particular case has symmetry of
    // revolution (meaning the result is independent of angle theta).
    // const double theta = M_PI / 5;
    CalcContactJacobian(q0(3), &data->J);
    {
      // A single block size.
      BlockSparseMatrixBuilder<double> builder(1, 1, 1);
      builder.PushBlock(0, 0, data->J);
      data->Jblock = builder.Build();
    }
    data->Jop =
        std::make_unique<BlockSparseLinearOperator<double>>("J", &data->Jblock);

    data->phi0.setConstant(num_contacts, q0(2));
    data->vc0.setConstant(3 * num_contacts, v0(2));  // TODO: needed?
    data->stiffness.setConstant(num_contacts, stiffness_);
    data->dissipation.setConstant(num_contacts, taud_ * stiffness_);
    data->mu.setConstant(num_contacts, mu_);

    data->contact_data = std::make_unique<PointContactData<double>>(
        &data->phi0, &data->vc0, data->Jop.get(), &data->stiffness,
        &data->dissipation, &data->mu);

    return data;
  }

#if 0
  // This tests the solver when we apply a moment Mz about COM to the pizza
  // saver. If Mz < mu * m * g * R, the saver should be in stiction (that is,
  // the sliding velocity should be smaller than the regularization parameter).
  // Otherwise the saver will start sliding. For this setup the transition
  // occurs at M_transition = mu * m * g * R = 5.0
  std::unique_ptr<ProblemData> MakeStictionProblemData(double dt) const {
    const double mu = 0.5;

    // External forcing.
    const double Mz = 3.0;  // M_transition = 5.0
    const Vector3<double> tau(0.0, 0.0, Mz);

    // Initial velocity.
    const Vector3<double> v0 = Vector3<double>::Zero();

    return MakeProblemData(dt, mu, tau, v0);
  }
#endif

  std::unique_ptr<ProblemData> MakeProblemData(const VectorXd& v0,
                                               double Mz) const {
    // Some arbitrary orientation. This particular case has symmetry of
    // revolution (meaning the result is independent of angle theta).
    const double theta = M_PI / 5;
    const Vector4d q0(0.0, 0.0, 0.0, theta);
    const Vector4d tau(0.0, 0.0, -m_ * g_, Mz);
    return MakeProblemData(q0, v0, tau);
  }

  std::unique_ptr<ProblemData> MakeProblemData(const VectorXd& v0, double fx,
                                               double Mz) const {
    // Some arbitrary orientation. This particular case has symmetry of
    // revolution (meaning the result is independent of angle theta).
    const double theta = M_PI / 5;
    const Vector4d q0(0.0, 0.0, 0.0, theta);
    const Vector4d tau(fx, 0.0, -m_ * g_, Mz);
    return MakeProblemData(q0, v0, tau);
  }

 private:
  double time_step_{NAN};

  // Pizza saver parameters.
  double m_{1.0};           // Mass of the pizza saver.
  double R_{1.0};           // Distance from COM to any contact point.
  double I_{R_ * R_ * m_};  // = 1.0 in this case.

  // Contact parameters:
  double mu_{0};           // Friction coefficient.
  double stiffness_{NAN};  // Contact stiffness k.
  double taud_{NAN};       // Linear dissipation time scale: c = taud * k.

  // Acceleration of gravity.
  const double g_{10.0};
};

ContactSolverResults<double> AdvanceNumSteps(
    const PizzaSaverProblem& problem, const VectorXd& tau, int num_steps,
    const SapSolverParameters& params) {
  SapSolver<double> sap;
  sap.set_parameters(params);
  ContactSolverResults<double> result;
  // Arbitrary non-zero guess to stress the solver.
  VectorXd v_guess(problem.num_velocities);
  v_guess << 1.0, 2.0, 3.0;

  const double theta = M_PI / 5;  // Arbitrary orientation.
  VectorXd q = Vector4d(0.0, 0.0, 0.0, theta);
  VectorXd v = VectorXd::Zero(problem.num_velocities);

  for (int i = 0; i < num_steps; ++i) {
    const auto data = problem.MakeProblemData(q, v, tau);
    const ContactSolverStatus status =
        sap.SolveWithGuess(problem.time_step(), *data->dynamics_data,
                           *data->contact_data, v_guess, &result);
    (void)status;
    // ASSERT_EQ(status, ContactSolverStatus::kSuccess);
    v = result.v_next;
    q += problem.time_step() * v;
  }

  return result;
}

GTEST_TEST(PizzaSaver, NoAppliedTorque) {
  const double dt = 0.01;
  const double mu = 1.0;
  const double k = 1.0e4;
  const double taud = dt;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.alpha = 0;  // Force SAP to use user stiffness.
  params.ls_method = SapSolverParameters::LineSearchMethod::kArmijo;
  params.ls_max_iterations = 40;

  const Vector4d tau(0.0, 0.0, -problem.mass() * problem.g(), 0.0);
  const ContactSolverResults<double> result =
      AdvanceNumSteps(problem, tau, 30, params);

  // N.B. The accuracy of the solutions is significantly higher when using exact
  // line search.
  // TODO(amcastro-tri): Tighten tolerances for runs with exact line search.

  // Expected generalized force.
  Vector4d tau_expected(0.0, 0.0, problem.mass() * problem.g(), 0.0);

  EXPECT_TRUE(CompareMatrices(result.v_next,
                              VectorXd::Zero(problem.num_velocities),
                              params.rel_tolerance));
  EXPECT_TRUE(CompareMatrices(result.tau_contact, tau_expected,
                              params.rel_tolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.ft,
                              VectorXd::Zero(2 * problem.num_contacts),
                              params.rel_tolerance));
  EXPECT_TRUE(CompareMatrices(
      result.fn,
      VectorXd::Constant(problem.num_contacts, tau_expected(2) / 3.0),
      params.rel_tolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.vt,
                              VectorXd::Zero(2 * problem.num_contacts),
                              params.rel_tolerance));
  EXPECT_TRUE(CompareMatrices(result.vn, VectorXd::Zero(problem.num_contacts),
                              params.rel_tolerance));
}

// This tests the solver when we apply a moment Mz about COM to the pizza
// saver. If Mz < mu * m * g * R, the saver should be in stiction (that is,
// the sliding velocity should be smaller than the regularization parameter).
// Otherwise the saver will start sliding. For this setup the transition
// occurs at M_transition = mu * m * g * R = 5.0
GTEST_TEST(PizzaSaver, Stiction) {
  const double dt = 0.01;
  const double mu = 1.0;
  const double k = 1.0e4;
  const double taud = dt;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.alpha = 0;  // Force SAP to use user stiffness.
  params.ls_method = SapSolverParameters::LineSearchMethod::kArmijo;
  params.ls_max_iterations = 40;

  const double Mz = 3.0;
  const Vector4d tau(0.0, 0.0, -problem.mass() * problem.g(), Mz);
  const ContactSolverResults<double> result =
      AdvanceNumSteps(problem, tau, 30, params);

  // Expected generalized force.
  const Vector4d tau_expected(0.0, 0.0, problem.mass() * problem.g(), -Mz);

  // Maximum expected slip. See Castro et al. 2021 for details.
  const double max_slip_expected = params.sigma * dt * problem.g();

  EXPECT_TRUE(CompareMatrices(result.v_next.head<3>(), Vector3d::Zero(),
                              params.abs_tolerance));
  EXPECT_TRUE(CompareMatrices(result.tau_contact, tau_expected,
                              params.rel_tolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      result.fn,
      VectorXd::Constant(problem.num_contacts, tau_expected(2) / 3.0),
      params.rel_tolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.vn, VectorXd::Zero(problem.num_contacts),
                              params.abs_tolerance));
  const double friction_force_expected = Mz / problem.radius() / 3.0;
  for (int i = 0; i < 3; ++i) {
    const double slip = result.vt.segment<2>(2 * i).norm();
    const double friction_force = result.ft.segment<2>(2 * i).norm();
    const double normal_force = result.fn(i);
    EXPECT_LE(friction_force, mu * normal_force);
    EXPECT_NEAR(friction_force, friction_force_expected, params.rel_tolerance);
    EXPECT_LE(slip, max_slip_expected);
  }
}

// Test case with zero friction coefficient.
GTEST_TEST(PizzaSaver, NoFriction) {
  const double dt = 0.01;
  const double mu = 0.0;
  const double k = 1.0e4;
  const double taud = dt;
  const int num_steps = 30;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.alpha = 0;  // Force SAP to use user stiffness.
  params.ls_method = SapSolverParameters::LineSearchMethod::kArmijo;
  params.ls_max_iterations = 40;

  const double fx = 1.0;
  const Vector4d tau(fx, 0.0, -problem.mass() * problem.g(), 0.0);
  const ContactSolverResults<double> result =
      AdvanceNumSteps(problem, tau, num_steps, params);

  // Expected generalized force.
  const Vector4d tau_expected(0.0, 0.0, problem.mass() * problem.g(), 0.0);

  const double vx_expected = num_steps * dt * fx / problem.mass();

  EXPECT_TRUE(CompareMatrices(result.v_next,
                              Vector4d(vx_expected, 0.0, 0.0, 0.0),
                              2.0 * params.rel_tolerance));
  EXPECT_TRUE(CompareMatrices(result.tau_contact, tau_expected,
                              2.0 * params.rel_tolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.ft,
                              VectorXd::Zero(2 * problem.num_contacts),
                              std::numeric_limits<double>::epsilon()));
  EXPECT_TRUE(CompareMatrices(
      result.fn,
      VectorXd::Constant(problem.num_contacts, tau_expected(2) / 3.0),
      2.0 * params.rel_tolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(result.vn, VectorXd::Zero(problem.num_contacts),
                              params.abs_tolerance));
  for (int i = 0; i < 3; ++i) {
    const auto vt = result.vt.segment<2>(2 * i);
    EXPECT_TRUE(CompareMatrices(vt, Vector2d(vx_expected, 0.0),
                                2.0 * params.rel_tolerance,
                                MatrixCompareType::relative));
  }
}

GTEST_TEST(PizzaSaver, Sliding) {
  const double dt = 0.01;
  const double mu = 0.5;
  const double k = 1.0e4;
  const double taud = dt;
  PizzaSaverProblem problem(dt, mu, k, taud);

  SapSolverParameters params;
  params.rel_tolerance = 1.0e-6;
  params.alpha = 0;  // Force SAP to use user stiffness.
  params.ls_method = SapSolverParameters::LineSearchMethod::kArmijo;
  params.ls_max_iterations = 40;

  const double Mz = 6.0;
  const double weight = problem.mass() * problem.g();
  const double fn_expected = weight / 3.0;
  const double ft_expected = mu * fn_expected;
  const double friction_torque_expected = 3.0 * ft_expected * problem.radius();
  const double I = problem.rotational_inertia();
  const double w_expected = dt * (Mz - friction_torque_expected) / I;
  const double slip_expected = w_expected * problem.radius();
  (void)slip_expected;

  // const double phi0 = dt * mu * slip_expected;
  // const Vector4d q0(0.0, 0.0, phi0, theta);
  // const VectorXd v0 = VectorXd::Zero(problem.num_velocities);
  const Vector4d tau(0.0, 0.0, -weight, Mz);

  const ContactSolverResults<double> result =
      AdvanceNumSteps(problem, tau, 10, params);

  // Expected generalized force.
  const Vector4d tau_expected(0.0, 0.0, problem.mass() * problem.g(),
                              -friction_torque_expected);

  EXPECT_GT(result.v_next(3), 0.0);  // It's accelerating.
  EXPECT_NEAR(tau_expected.head<2>().norm(), 0.0, params.abs_tolerance);
  EXPECT_GT(std::abs(tau_expected(2)), 0.0);
  EXPECT_LT(std::abs(tau_expected(3)), Mz);
  for (int i = 0; i < 3; ++i) {
    const double friction_force = result.ft.segment<2>(2 * i).norm();
    const double normal_force = result.fn(i);
    EXPECT_NEAR(friction_force, mu * normal_force,
                5.0 * std::numeric_limits<double>::epsilon());
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
