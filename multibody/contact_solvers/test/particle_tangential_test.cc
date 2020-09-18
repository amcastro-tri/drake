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

namespace drake {
namespace multibody {

using test::MultibodySimDriver;

namespace contact_solvers {
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
class ParticleTest : public ::testing::Test {
 public:
  //void SetUp() override { SetUp(Vector3d::UnitZ()); }

  void SetParticleWithASingleMotionAxis(const Vector3d& axis) {
    const std::string model_file =
        "drake/multibody/contact_solvers/test/particle.sdf";
    driver_.BuildModel(dt_, model_file);
    auto& plant = driver_.mutable_plant();
    const auto& particle = plant.GetBodyByName("particle");

    // Allow a single dof only.
    // Pose of slider S in the world slightly below the ground so that we do
    // have contact.
    const RigidTransformd X_WS(Vector3d(0, 0, kSignedDistance_));
    plant.AddJoint<PrismaticJoint>("slider", plant.world_body(), X_WS, particle,
                                   {}, axis);

    // Set stiffness so that at the fixed penetration we set we do have a
    // non-zero normal force, even if motions can only happen in the tangential
    // direction.
    const double weight = 5.0;  // mass = 0.5 Kg and g = 10.0 m/s².
    const double k = -weight / this->kSignedDistance_;
    PRINT_VAR(k);
    this->driver_.SetPointContactParameters(particle, k, 0.0);

    // Add the ground, with the same friction as specified in the SDF file for
    // the particle.
    const std::vector<double> geometry_mu =
        driver_.GetDynamicFrictionCoefficients(particle);
    ASSERT_EQ(geometry_mu.size(), 1u);

    // We make the ground infinitely stiff.
    driver_.AddGround(1.0e20, 0.0, geometry_mu[0]);
    
    driver_.Initialize();
    const int nq = plant.num_positions();
    const int nv = plant.num_velocities();
    // Assert plant sizes.
    ASSERT_EQ(nq, 1);
    ASSERT_EQ(nv, 1);
    solver_ = &driver_.mutable_plant().set_contact_solver(
        std::make_unique<ContactSolverType>());

    // MultibodyPlant state.
    //SetInitialState();
  }

  // Set the particle to be in contact with the ground.
  void SetInitialState() {
    const auto& plant = driver_.plant();
    const auto& body = plant.GetBodyByName("particle");
    const auto& slider = plant.GetJointByName<PrismaticJoint>("slider");
    auto& context = driver_.mutable_plant_context();
    slider.set_translation(&context, kSignedDistance_);
    slider.set_translation_rate(&context, 0.0);
  }

  VectorXd EvalGeneralizedContactForces() const {
    const auto& body = driver_.plant().GetBodyByName("particle");
    const VectorXd tau_c = driver_.plant()
                           .get_generalized_contact_forces_output_port(
                               body.model_instance())
                           .Eval(driver_.plant_context());
    return tau_c;
  }

  void SetParams(NormalConstraintType,
                 test::PgsSolver<double>* solver) const {
    test::PgsSolverParameters params; 
    // PGS diverges for this case if relaxation > 0.6. Therefore we use half of
    // that and we observe convergence in 8 iterations.
    // A small change to relaxation = 0.2 leads to 18 iterations.
    params.relaxation = 0.2;
    params.max_iterations = 500;
    params.abs_tolerance = 1.0e-8;
    params.rel_tolerance = 1.0e-6;
    solver->set_parameters(params);
  }

  void PrintStats(const test::PgsSolver<double>& solver) const {
    auto& stats = solver.get_iteration_stats();
    PRINT_VAR(stats.iterations);
    PRINT_VAR(stats.vc_err);
    PRINT_VAR(stats.gamma_err);
  }

  void SetParams(NormalConstraintType type,
                 MacklinSolver<double>* solver) const {
    MacklinSolverParameters params;
    params.max_iters = 50;
    params.absolute_tolerance = 1.0e-8;
    params.relative_tolerance = 1.0e-6;
    params.stiction_tolerance = 1.0e-7;
    params.macklin_jacobian = false;
    // For this simple test, the solver converges to the solution in one single
    // iteration. Since we start with a guess pi > 0 and vn = 0, then we have
    // that constraint gn = fFB(vn, pi) = 0 is satisfied.
    // Therefore on the very first iteration the problem reduces to simply
    // solving the linear momentum equation v = v_star + pi / m = 0.
    // This will not be true in general and we'll need a non-zero relaxation.
    params.relaxation = 1.0;

    // Normal constraint type.
    params.alpha_stab = type == NormalConstraintType::kVelocity ? -1.0 : 1.0;
    
    solver->set_parameters(params);
  }

  void PrintStats(const MacklinSolver<double>& solver) const {
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
  const double kSignedDistance_{-1.0e-3};
  MultibodySimDriver driver_;
  ContactSolverType* solver_{nullptr};
  const double kContactVelocityAbsoluteTolerance_{1.0e-5};  // m/s.
  const double kContactVelocityRelativeTolerance_{1.0e-6};  // Unitless.
};

TYPED_TEST_SUITE_P(ParticleTest);

TYPED_TEST_P(ParticleTest, Stiction) {
  this->SetParticleWithASingleMotionAxis(Vector3d::UnitX());

  // External forcing.
  const double Fx = 1.0;  // Fx_transition = 2.5
  const SpatialForce<double> F_Bo_W(Vector3d::Zero(), Vector3d(Fx, 0, 0));
  const auto& particle = this->driver_.plant().GetBodyByName("particle");
  this->driver_.FixAppliedForce(particle, F_Bo_W);  

  TypeParam& solver = *this->solver_;
  this->SetParams(NormalConstraintType::kPosition, &solver);

  // Verify forces in static equilibrium.
  // TODO(amcastro-tri): allow acces to scaling diagonals of W from
  // ContactSolver so that we can convert units.
  // Here we are assuming Wii = 1.0, which is OK for this problem.
  const VectorXd tau_c = this->EvalGeneralizedContactForces();
  const VectorXd& vc = solver.GetContactVelocities();
  PRINT_VAR(tau_c.transpose());
  PRINT_VAR(vc.transpose());

  this->PrintStats(solver);
}

TYPED_TEST_P(ParticleTest, Sliding) {
  this->SetParticleWithASingleMotionAxis(Vector3d::UnitX());

  // External forcing.
  const double Fx = 4.0;  // Fx_transition = 2.5
  const SpatialForce<double> F_Bo_W(Vector3d::Zero(), Vector3d(Fx, 0, 0));
  const auto& particle = this->driver_.plant().GetBodyByName("particle");
  this->driver_.FixAppliedForce(particle, F_Bo_W);  

  TypeParam& solver = *this->solver_;
  this->SetParams(NormalConstraintType::kPosition, &solver);

  // Verify forces in static equilibrium.
  // TODO(amcastro-tri): allow acces to scaling diagonals of W from
  // ContactSolver so that we can convert units.
  // Here we are assuming Wii = 1.0, which is OK for this problem.
  const VectorXd tau_c = this->EvalGeneralizedContactForces();
  const VectorXd& vc = solver.GetContactVelocities();
  PRINT_VAR(tau_c.transpose());
  PRINT_VAR(vc.transpose());

  this->PrintStats(solver);
}

REGISTER_TYPED_TEST_SUITE_P(ParticleTest, Stiction, Sliding);

typedef ::testing::Types<test::PgsSolver<double>, MacklinSolver<double>>
    ContactSolverTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(ContactSolvers, ParticleTest,
                               ContactSolverTypes);

}  // namespace
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake


