#include <iostream>
#include <memory>

#include <fstream>

#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/macklin_solver.h"
#include "drake/multibody/contact_solvers/test/multibody_sim_driver.h"
#include "drake/multibody/contact_solvers/test/pgs_solver.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/tree/prismatic_joint.h"

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace multibody {

using test::MultibodySimDriver;

namespace contact_solvers {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

enum class NormalConstraintType {
  /// Normal constraint imposes vn = 0.
  kVelocity = 0,

  /// Normal constraint imposes phi = 0.
  kPosition = 1,
};

template <typename ContactSolverType>
class StackOfBoxesTest : public ::testing::Test {
 public:
  //void SetUp() override { SetUp(Vector3d::UnitZ()); }

  void SetUp() {
    const std::string model_file =
        "drake/multibody/contact_solvers/test/stack_of_boxes.sdf";
    driver_.BuildModel(dt_, model_file);
    auto& plant = driver_.mutable_plant();

    // TODO: assert that this value of friction is what we have in the SDF.
    const double mu = 0.5;

    // We make the ground infinitely stiff.
    driver_.AddGround(1.0e20, 0.0, mu);

    // Driver will add systems for viz and contact results. No publish will be
    // triggered. A Context is available after this call.
    driver_.Finalize();
    SetInitialState();
    const int nq = plant.num_positions();
    const int nv = plant.num_velocities();

    // Assert plant sizes.
    //ASSERT_EQ(nq, 35);
    //ASSERT_EQ(nv, 30);
    solver_ = &driver_.mutable_plant().set_contact_solver(
        std::make_unique<ContactSolverType>());
  }

  // Set the particle to be in contact with the ground.
  void SetInitialState() {
    const auto& plant = driver_.plant();
    auto& context = driver_.mutable_plant_context();
    double stack_height = 0;
    for (int i = 1; i < plant.num_bodies(); ++i) {
      std::string body_name = "Object" + std::to_string(i);
      const auto& body = plant.GetBodyByName(body_name);
      const double length = 0.2 * std::pow(2.0, i-1);
      RigidTransformd X_WB(Vector3d(
          0.0, 0.0, stack_height + length / 2.0 - kInitialPenetration_));
      plant.SetFreeBodyPose(&context, body, X_WB);
      stack_height += (length - kInitialPenetration_);
      PRINT_VAR(stack_height);
      PRINT_VAR(X_WB.translation().transpose());
    }
#if 0    
    const auto& plant = driver_.plant();
    const auto& body = plant.GetBodyByName("particle");
    const auto& slider = plant.GetJointByName<PrismaticJoint>("slider");
    auto& context = driver_.mutable_plant_context();
    slider.set_translation(&context, kSignedDistance_);
    slider.set_translation_rate(&context, 0.0);
#endif    
  }

  VectorXd EvalGeneralizedContactForces() const {
    const VectorXd tau_c =
        driver_.plant()
            .get_generalized_contact_forces_output_port(
                driver_.plant().GetModelInstanceByName("StackOfObjects"))
            .Eval(driver_.plant_context());
    return tau_c;
  }

  //void SetGuess(const VectorXd& v, const VectorXd& gamma,
  //              PgsSolver<double>* solver) const {

//}

  void SetParams(NormalConstraintType,
                 PgsSolver<double>* solver) const {
    PgsSolverParameters params; 
    params.relaxation = 1;
    params.max_iterations = 25000;
    params.abs_tolerance = 1.0e-4;
    params.rel_tolerance = 1.0e-5;
    solver->set_parameters(params);
  }

  void PrintStats(const PgsSolver<double>& solver) const {
    auto& stats = solver.get_iteration_stats();
    std::cout << std::string(80, '-') << std::endl;  
    std::cout << std::string(80, '-') << std::endl;

    PRINT_VAR(stats.iterations);
    fmt::print("{:>18} {:>18}\n", "vc", "gamma");
    for (const auto& errors : stats.iteration_errors) {
      fmt::print("{:18.6g} {:18.6g}\n", errors.vc_err, errors.gamma_err);
    }
  }

  void SetParams(NormalConstraintType type,
                 MacklinSolver<double>* solver) const {
    MacklinSolverParameters params;
    params.max_iters = 100;
    params.absolute_tolerance = 1.0e-8;
    params.relative_tolerance = 1.0e-6;
    params.stiction_tolerance = 1.0e-7;
    params.macklin_jacobian = false;
    params.relaxation = 1.0;

    // Normal constraint type.
    params.alpha_stab = type == NormalConstraintType::kVelocity ? -1.0 : 1.0;
    
    solver->set_parameters(params);
  }

  void PrintStats(const MacklinSolver<double>& solver) const {
    const auto& stats_hist = solver.get_stats_history();
    PRINT_VAR(stats_hist.size());
    // We want to make sure we absolutely know the number of solves.
    //EXPECT_EQ(stats_hist.size(), 1u);

    const MacklinSolverIterationStats& stats = solver.get_stats();

    std::cout << std::string(80, '-') << std::endl;  
    std::cout << std::string(80, '-') << std::endl;

    PRINT_VAR(stats.iterations);
    //std::cout << "momentum - MDP - normal_slackness - tangential_slackness\n";
    // Fixed width of 12, right aligned.
    // NB. < for left aligned and ^ for centered.
    fmt::print("{:>18} {:>18} {:>18} {:>18}\n", "momentum", "MDP",
               "normal_slack", "tangential_slack");
    for (const auto& errors : stats.iteration_errors) {
      fmt::print("{:18.6g} {:18.6g} {:18.6g} {:18.6g}\n", errors.momentum_error,
                 errors.mdp_error, errors.normal_slackness_error,
                 errors.tangential_slackness_error);
    }
  }

 protected:
  const double dt_{1.0e-3};
  const double kInitialPenetration_{1.0e-15};
  MultibodySimDriver driver_;
  ContactSolverType* solver_{nullptr};
  const double kContactVelocityAbsoluteTolerance_{1.0e-5};  // m/s.
  const double kContactVelocityRelativeTolerance_{1.0e-6};  // Unitless.
};

TYPED_TEST_SUITE_P(StackOfBoxesTest);

TYPED_TEST_P(StackOfBoxesTest, Static) {
  TypeParam& solver = *this->solver_;
  this->SetParams(NormalConstraintType::kVelocity, &solver);

  const auto& context  = this->driver_.plant_context();
  PRINT_VAR(
      context.get_discrete_state().get_vector().CopyToVector().transpose());  

  //const VectorXd v_guess = VectorXd::Zero(1);
  //VectorXd gamma_guess = VectorXd::Zero(3);
  //gamma_guess(2) = 5.0 * this->dt_;
  //solver.set_guess(v_guess, gamma_guess);

  // Verify forces in static equilibrium.
  // TODO(amcastro-tri): allow acces to scaling diagonals of W from
  // ContactSolver so that we can convert units.
  // Here we are assuming Wii = 1.0, which is OK for this problem.
  const VectorXd tau_c = this->EvalGeneralizedContactForces();
  const VectorXd& vc = solver.GetContactVelocities();
  PRINT_VAR(tau_c.transpose());
  PRINT_VAR(vc.transpose());

  // For visualization.
  this->driver_.Publish();

  this->PrintStats(solver);
}

REGISTER_TYPED_TEST_SUITE_P(StackOfBoxesTest, Static);

typedef ::testing::Types<PgsSolver<double>, MacklinSolver<double>>
    ContactSolverTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(ContactSolvers, StackOfBoxesTest,
                               ContactSolverTypes);

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
