#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// clang-format off
const Eigen::Matrix2d S22 =
    (Eigen::Matrix2d() << 2, 1,
                          1, 2).finished();
const Eigen::Matrix3d S33 =
    (Eigen::Matrix3d() << 4, 1, 2,
                          1, 5, 3,
                          2, 3, 6).finished();
const Eigen::Matrix4d S44 =
    (Eigen::Matrix4d() << 7, 1, 2, 3,
                          1, 8, 4, 5,
                          2, 4, 9, 6,
                          3, 5, 6, 10).finished();
// clang-format on

// clang-format off
const MatrixXd J31 =
    (MatrixXd(3, 1) << 1,
                       2,
                       3).finished();

const MatrixXd J32 =
    (MatrixXd(3, 2) << 2, 1,
                       1, 2,
                       1, 2).finished();
const MatrixXd J33 =
    (MatrixXd(3, 3) << 4, 1, 2,
                       1, 5, 3,
                       2, 3, 6).finished();
const MatrixXd J34 =
    (MatrixXd(3, 4) << 7, 1, 2, 3,
                       1, 8, 4, 5,
                       2, 4, 9, 6).finished();
// clang-format on

const MatrixXd Z31 = MatrixXd::Zero(3, 1);
const MatrixXd Z32 = MatrixXd::Zero(3, 2);
const MatrixXd Z34 = MatrixXd::Zero(3, 4);

namespace {

// With SAP we can model implicit springs using constraints. For these
// constraint the projection is the identity, i.e. γ = P(y) = y.
// For testing purposes, this is a simple constraints that models a spring
// between a particle mass and the origin. The spring has stiffness k and
// damping d = tau_d * k, where tau_d is the dissipation time scale. That is,
// the force applied by this constraint on the mass is γ/δt = −k⋅x − d⋅v, where
// x is the (3D) position of the mass and v its (3D) velocity.
template <typename T>
class SpringConstraint final : public SapConstraint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpringConstraint);

  // Model a spring attached to `clique`, expected to be a 3D particle.
  explicit SpringConstraint(int clique, VectorX<T> x, T k, T tau_d)
      // N.B. For this constraint the Jacobian is the identity matrix.
      : SapConstraint<T>(clique, std::move(x), Matrix3<T>::Identity()) {}

  // Bias and regularization setup so that: 
  //   γ = y = -δt⋅(k⋅x + d⋅v) = −R⁻¹⋅(v−v̂).
  VectorX<T> CalcBiasTerm(const T& time_step, const T&) const final {
    return -this->constraint_function() / (time_step + tau_d_);
  }
  VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                        const T&) const final {
    const T R = 1. / (time_step * (time_step + tau_d_) * k_);
    return Vector3<T>(R, R, R);
  }

  // For this constraint the projection is the identity operation.
  void Project(const Eigen::Ref<const VectorX<double>>& y,
               const Eigen::Ref<const VectorX<double>>& R,
               EigenPtr<VectorX<double>> gamma,
               MatrixX<double>* dPdy) const final {
    (*gamma) = y;
    if (dPdy != nullptr) dPdy->setIdentity(3, 3);
  };

 private:
  T k_{0.0};      // Stiffness, in N/m.
  T tau_d_{0.0};  // Dissipation time scale, in N⋅s/m.
};

// Sets up a simple problem for two 3D particles, six DOFs.
// The first mass is connected to the origin by a spring-damper while the second
// mass is free.
// The problem is setup as two distinct cliques, one for each mass. The
// spring-damper is modeled as a SpringConstraint.
// Since only the first mass is constrained, we know that the SapModel will only
// consider the dynamics of the first mass connected to the origin by the
// spring-damper.
template <typename T>
class SpringMassModel {
 public:
  SpringMassModel() = default;

  // Data accessors.
  double mass1() const { return mass1_; }
  double mass2() const { return mass2_; }
  double time_step() const { return time_step_; }
  double gravity() const { return gravity_; }

  // Make a SapContactProblem for this model at the state described by positions
  // q and velocities v. q.head<3>() and v.head<3>() correspond to the state for
  // the first mass while q.tail<3>() and v.tail<3>() correspond to the state
  // for the second mass.
  std::unique_ptr<SapContactProblem<T>> MakeContactProblem(
      const VectorX<T>& q, const VectorX<T>& v) {
    DRAKE_DEMAND(q.size() == 6);
    DRAKE_DEMAND(v.size() == 6);
    std::vector<MatrixX<T>> A = {mass1_ * Matrix3<T>::Identity(),
                                 mass2_ * Matrix3<T>::Identity()};
    const Vector6<T> g =
        gravity_ *
        (Vector6<T>() << Vector3<T>::UnitZ(), Vector3<T>::UnitZ()).finished();
    VectorX<T> v_star = v - time_step_ * g;
    auto problem = std::make_unique<SapContactProblem<T>>(
        time_step_, std::move(A), std::move(v_star));
    problem->AddConstraint(std::make_unique<SpringConstraint<T>>(
        0, q.template head<3>(), stiffness_, dissipation_time_scale_));
    return problem;
  }

 protected:
  double time_step_{1.0e-3};
  double mass1_{1.5};
  double mass2_{3.0};
  double stiffness_{100.0};
  double dissipation_time_scale_{0.1};
  double gravity_{10.0};
};

// This fixture setsup a SpringMassModel to test SapModel.
class SpringMassTest : public ::testing::Test {
 public:
  void SetUp() override { MakeModel(Vector6d::Zero(), Vector6d::Zero()); }
  void MakeModel(const VectorXd& q0, const VectorXd& v0) {    
    sap_problem_ = model_.MakeContactProblem(q0, v0);
    // Sanity check problem sizes.
    EXPECT_EQ(sap_problem_->num_cliques(), 2);
    EXPECT_EQ(sap_problem_->num_velocities(), 6);
    EXPECT_EQ(sap_problem_->num_constraints(), 1);
    EXPECT_EQ(sap_problem_->num_constraint_equations(), 3);
    sap_model_ = std::make_unique<SapModel<double>>(sap_problem_.get());
    context_ = sap_model_->MakeContext();
  }

 protected:
  SpringMassModel<double> model_;
  std::unique_ptr<SapContactProblem<double>> sap_problem_;
  std::unique_ptr<SapModel<double>> sap_model_;
  std::unique_ptr<systems::Context<double>> context_;
};

TEST_F(SpringMassTest, Sizes) {
  // While the SapProblem has two cliques, six velocities, in the SapModel only
  // one clique with three velocities participates. The second clique is not
  // connected by any constraint.
  EXPECT_EQ(sap_model_->num_cliques(), 1);
  EXPECT_EQ(sap_model_->num_velocities(), 3);
  EXPECT_EQ(sap_model_->num_constraints(), 1);
  EXPECT_EQ(sap_model_->num_constraint_equations(), 3);
}

// Since only the first clique participates, we expect the problem data to
// correspond to that of the first spring only.
TEST_F(SpringMassTest, ProblemData) {
  // We setup the problem with two distinct initial velocities so that v* for
  // each clique is also distinct and we can tell them appart.
  const Vector3d v1(1., 2., 3.);
  const Vector3d v2(4., 5., 6.);
  const Vector6d v = (Vector6d() << v1, v2).finished();
  MakeModel(Vector6d::Zero(), v);
  EXPECT_EQ(sap_model_->time_step(), sap_problem_->time_step());
  // We expect the mass matrix for the first mass only since it is the only one
  // connected by a constraint.
  const std::vector<MatrixXd> A = {model_.mass1() * Matrix3d::Identity()};
  EXPECT_EQ(sap_model_->dynamics_matrix(), A);
  const Vector3d v_star =
      v1 - model_.time_step() * model_.gravity() * Vector3d::UnitZ();
  EXPECT_EQ(sap_model_->v_star(), v_star);
  const Vector3d p_star = model_.mass1() * v_star;
  EXPECT_EQ(sap_model_->p_star(), p_star);
}

TEST_F(SpringMassTest, StateAccess) {
  const Vector3d v(1., 2., 3.);
  sap_model_->SetVelocities(v, context_.get());
  EXPECT_EQ(sap_model_->GetVelocities(*context_), v);
}

TEST_F(SpringMassTest, EvalConstraintVelocities) {
  const Vector3d v(1., 2., 3.);
  sap_model_->SetVelocities(v, context_.get());
  EXPECT_EQ(sap_model_->EvalConstraintVelocities(*context_), v);
}

TEST_F(SpringMassTest, EvalMomentumGain) {
  const Vector3d v(1., 2., 3.);
  sap_model_->SetVelocities(v, context_.get());
  const VectorXd& v_star = sap_model_->v_star();
  const VectorXd momentum_gain = model_.mass1() * (v - v_star);  
  EXPECT_EQ(sap_model_->EvalMomentumGain(*context_), momentum_gain);
}

TEST_F(SpringMassTest, EvalMomentumCost) {
  const Vector3d v(1., 2., 3.);
  sap_model_->SetVelocities(v, context_.get());
  const VectorXd& v_star = sap_model_->v_star();
  const double momentum_cost = 0.5 * model_.mass1() * (v - v_star).squaredNorm();  
  EXPECT_EQ(sap_model_->EvalMomentumCost(*context_), momentum_cost);
}

#if 0
class TestConstraint final : public SapConstraint<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestConstraint);

  TestConstraint(int clique, double param)
      : SapConstraint<double>(clique, Vector3d(1., 2., 3.),
                              MakeJacobianMatrix(clique)),
        param_(param) {}

  // @throws if the number of rows in J0 and J1 is different from three.
  TestConstraint(int clique0, int clique1, double param)
      : SapConstraint<double>(clique0, clique1, Vector3d(1., 2., 3.),
                              MakeJacobianMatrix(clique0),
                              MakeJacobianMatrix(clique1)),
        param_(param) {}

  VectorX<double> CalcBiasTerm(const double&, const double&) const final {
    return Eigen::Vector3d::Ones() * param_;
  }

  VectorX<double> CalcDiagonalRegularization(const double& time_step,
                                             const double& wi) const final {
    return Eigen::Vector3d::Ones() * param_ * 2.0;
  }

  // Not used in these tests.
  void Project(const Eigen::Ref<const VectorX<double>>&,
               const Eigen::Ref<const VectorX<double>>&,
               EigenPtr<VectorX<double>>, MatrixX<double>*) const final {
    DRAKE_UNREACHABLE();
  };

 private:
  // This class constraints 3 DOFs. Therefore Jacobian matrices always have 3
  // rows. For this test, the dimension of the dynamics matrix is the clique
  // index plus one. Therefore this helper methods generates an arbitrary
  // Jacobian matrix with the proper size for a given clique index. That is,
  // while sizes are consistent with the problem in this test, numeric values
  // are arbitrary.
  MatrixXd MakeJacobianMatrix(int clique) {
    switch (clique) {
      case 0:
        return J31;
        break;
      case 1:
        return J32;
        break;
      case 2:
        return J33;
        break;
      case 3:
        return J34;
        break;
      default:
        throw std::runtime_error("More than 4 DOFs not supported.");
    }
  }

  double param_{0.0};  
};

class SapModelTester : public ::testing::Test {
 public:
  // Creates MultibodyPlant for an acrobot model.
  void SetUp() override {
    const double time_step = 0.01;

    // Setup a problem with four cliques and linear dynamics given by
    // A = {I₁, 2I₂, 3I₃, 4I₄}.
    // Therefore the number of generalized velocities is nv = 10.
    const int num_cliques = 4;
    A_.resize(num_cliques);
    for (int c = 1; c <= num_cliques; ++c) {
      A_[c - 1].resize(c, c);
      A_[c - 1] = MatrixXd::Identity(c, c) * c;
    }
    const int num_velocities = 10;
    v_star_ = VectorXd::LinSpaced(num_velocities, 1.0, 10.0);

    // N.B. We create copies of A and v_star so that we can keep the originals
    // during the unit test (SapContactProblem's constructor moves the data).
    problem_ = std::make_unique<SapContactProblem<double>>(
        time_step, std::vector<MatrixXd>(A_), VectorXd(v_star_));

    // Make a problem using contact constraints with the following graph:
    //
    //                       2
    //                      ┌─┐
    //                      │ │
    // ┌───┐  0  ┌───┐  1  ┌┴─┴┐     ┌───┐
    // │ 0 ├─────┤ 1 ├─────┤ 3 │     │ 2 │
    // └─┬─┘     └───┘     └─┬─┘     └───┘
    //   │                   │
    //   └───────────────────┘
    //           3,4
    //
    problem_->AddConstraint(std::make_unique<TestConstraint>(0, 1, 1.0));
    problem_->AddConstraint(std::make_unique<TestConstraint>(1, 3, 2.0));
    problem_->AddConstraint(std::make_unique<TestConstraint>(3, 3.0));
    problem_->AddConstraint(std::make_unique<TestConstraint>(0, 3, 4.0));
    problem_->AddConstraint(std::make_unique<TestConstraint>(0, 3, 5.0));
  }

  static const SapConstraintBundle<double>& constraints_bundle(
      const SapModel<double>& model) {
    DRAKE_DEMAND(model.constraints_bundle_ != nullptr);
    return *model.constraints_bundle_;
  }

 protected:
  // Data used to define the linearized dynamics of the system, i.e.
  // A⋅(v−v*) = Jᵀ⋅γ.
  std::vector<MatrixXd> A_;  // Linear system dynamics.
  VectorXd v_star_;          // Free-motion generalized velocities.
  std::unique_ptr<SapContactProblem<double>> problem_;
};

namespace {

TEST_F(SapModelTester, VerifySizes) {
  EXPECT_EQ(problem_->num_cliques(), 4);
  EXPECT_EQ(problem_->num_velocities(), 10);
  EXPECT_EQ(problem_->num_velocities(0), 1);
  EXPECT_EQ(problem_->num_velocities(1), 2);
  EXPECT_EQ(problem_->num_velocities(2), 3);
  EXPECT_EQ(problem_->num_velocities(3), 4);
  EXPECT_EQ(problem_->num_constraints(), 5);
  EXPECT_EQ(problem_->num_constraint_equations(), 15);

  PRINT_VAR(problem_->num_cliques());
  PRINT_VAR(problem_->num_velocities());
  PRINT_VAR(problem_->num_constraints());
  PRINT_VAR(problem_->num_constraint_equations());

  const SapModel<double> model(problem_.get());
  EXPECT_EQ(model.num_cliques(), 3);
  EXPECT_EQ(model.num_constraints(), 5);
  EXPECT_EQ(model.num_velocities(), 7);
  EXPECT_EQ(model.num_constraint_equations(), 15);
}

// Verify participating cliques are numbered according to the order they were
// defined originally. That is, we expect participating cliques to be
// re-numbered as: 0 --> 0, 1 --> 1, 3 --> 2.
TEST_F(SapModelTester, VerifyPermutations) {
  const SapModel<double> model(problem_.get());
  const std::vector<int> cliques_permutation_expected{0, 1, -1, 2};
  EXPECT_EQ(model.cliques_permutation().permutation(),
            cliques_permutation_expected);

  // Verify participating velocities permutation.
  // clang-format off
  const std::vector<int> velocities_permutation_expected{
    0,              // clique 0
    1, 2,           // clique 1
    -1, -1, -1,     // clique 2 (does not participate)
    3, 4, 5,  6};   // clique 3
  // clang-format on
  EXPECT_EQ(model.velocities_permutation().permutation(),
            velocities_permutation_expected);

  // Clique pair  |  Constraint |  Indexes
  //     (0, 1)   |  0          |  0, 1, 2
  //     (0, 3)   |  3          |  3, 4, 5
  //     (0, 3)   |  4          |  6, 7, 8
  //     (1, 3)   |  1          |  9, 10, 11
  //     (3, 3)   |  2          |  12, 13, 14
  // clang-format off
  const std::vector<int> impulses_permutation_expected{
    0, 1, 2,     // constraint 0
    9, 10, 11,   // constraint 1
    12, 13, 14,  // constraint 2
    3, 4, 5,     // constraint 3
    6, 7, 8};    // constraint 4
  // clang-format on
  EXPECT_EQ(model.impulses_permutation().permutation(),
            impulses_permutation_expected);
}

// Unit test to verify dynamics quantities related to participating DOFs only.
TEST_F(SapModelTester, VerifyParticipatingDofsDynamics) {
  const SapModel<double> model(problem_.get());

  // N.B. We know that participating cliques are enumerated according to the
  // order they were enumerated in the original cliques.
  std::vector<MatrixXd> A_permuted_expected;
  A_permuted_expected.push_back(A_[0]);
  A_permuted_expected.push_back(A_[1]);
  A_permuted_expected.push_back(A_[3]);

  for (const auto& Ac : A_permuted_expected) {
    PRINT_VARn(Ac);
  }

  EXPECT_EQ(model.dynamics_matrix(), A_permuted_expected);

  PRINT_VAR(model.num_cliques());
  PRINT_VAR(model.num_constraints());
  PRINT_VAR(model.num_velocities());
  PRINT_VAR(model.num_constraint_equations());

  // Verify v_star.
  // Clique 2 with velocities (4, 5, 6) does not participate and therefore it
  // does not show up in v_start_participating.
  const VectorXd v_star_participating_expected =
      (VectorXd(7) << 1., 2., 3., 7., 8., 9., 10.).finished();
  EXPECT_EQ(model.v_star(), v_star_participating_expected);

  // Verify p_star = A * v (participating DOFs only).
  const VectorXd p_star_participating_expected =
      (VectorXd(7) << 1., 4., 6., 28., 32., 36., 40.).finished();
  EXPECT_EQ(model.p_star(), p_star_participating_expected);

  // Unit test MultiplyByDynamicsMatrix().
  const VectorXd v = VectorXd::LinSpaced(10, 1.0, 10.0);
  VectorXd v_participating(model.num_cliques());
  model.velocities_permutation().Apply(v, &v_participating);
  PRINT_VARn(v.transpose());
  PRINT_VARn(v_participating.transpose());

  VectorXd p_participating(model.num_cliques());
  model.MultiplyByDynamicsMatrix(v_participating, &p_participating);

  // clang-format off
  const VectorXd p_participating_expected = (VectorXd(7) << 
    1,                           // clique 0. 1 x 1.
    4, 6,                        // clique 1. 1 x (2, 3).
    28, 32, 36, 40).finished();  // clique 3. 4 x (7, 8, 9, 10).
  // clang-format on
  EXPECT_TRUE(CompareMatrices(p_participating, p_participating_expected));
  PRINT_VARn(p_participating.transpose());
}

TEST_F(SapModelTester, VerifyConstraintsBundleJacobian) {
  const SapModel<double> model(problem_.get());

  EXPECT_EQ(constraints_bundle(model).num_constraints(), 5);

  // clang-format off
  const MatrixXd J_expected = (MatrixXd(15, 7) <<
    J31, J32, Z34,
    J31, Z32, J34,
    J31, Z32, J34,
    Z31, J32, J34,
    Z31, Z32, J34).finished();
  // clang-format on
  PRINT_VARn(J_expected);

  const MatrixXd J = constraints_bundle(model).J().MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(J, J_expected));

  PRINT_VARn(J);
}

TEST_F(SapModelTester, VerifyConstraintBiasAndRegularization) {
  const SapModel<double> model(problem_.get());

  // SapModel organizes the constraints bundle in the order specified by the
  // ContactProblemGraph of the specified SapContactProblem. We know the graph
  // groups constraints lexicographically by clique pairs. Within each group
  // constraints are in the order they were added. Therefore we know constraints
  // are in the following new order dictated by the graph:
  //
  // Clique pair  |  Constraint index (see schematic in the fixture)
  //     (0, 1)   |  0
  //     (0, 3)   |  3
  //     (0, 3)   |  4
  //     (1, 3)   |  1
  //     (3, 3)   |  2

  // Based on the reordering of constraints documented above, and given the
  // implementation of TestConstraint::CalcBiasTerm(), we know the expected vhat
  // is:
  // clang-format off
  const VectorXd vhat_expected = (VectorXd(15) << 
    Vector3d::Ones() * 1.0,
    Vector3d::Ones() * 4.0,
    Vector3d::Ones() * 5.0,
    Vector3d::Ones() * 2.0,
    Vector3d::Ones() * 3.0
  ).finished();
  // clang-format on

  EXPECT_TRUE(CompareMatrices(constraints_bundle(model).vhat(), vhat_expected));

  // Based on the reordering of constraints documented above, and given the
  // implementation of TestConstraint::CalcDiagonalRegularization(), we know the
  // expected regularization diagonal matrix R is:
  // clang-format off
  const VectorXd R_expected = (VectorXd(15) << 
    Vector3d::Ones() * 2.0,
    Vector3d::Ones() * 8.0,
    Vector3d::Ones() * 10.0,
    Vector3d::Ones() * 4.0,
    Vector3d::Ones() * 6.0
  ).finished();
  // clang-format on

  EXPECT_TRUE(CompareMatrices(constraints_bundle(model).R(), R_expected));
}
#endif

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake