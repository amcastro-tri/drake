#include <chrono>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/sap_driver.h"
#include "drake/multibody/plant/test/compliant_contact_manager_tester.h"
#include "drake/visualization/visualization_config_functions.h"

using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverParameters;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using drake::multibody::contact_solvers::internal::SapSolverStatus;
using drake::multibody::internal::DiscreteContactPair;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Tests that SolveWithGuess returns the free motion velocities and zero
// impulses if the constraints are empty.
GTEST_TEST(SapAutoDiffTest, SolveWithGuess) {
  constexpr double kTimeStep = 0.001;
  constexpr int kNumDofs = 5;
  constexpr int kNumCliques = 1;
  std::vector<MatrixX<AutoDiffXd>> A(kNumCliques);
  A[0] = MatrixX<AutoDiffXd>::Identity(kNumDofs, kNumDofs);
  VectorX<AutoDiffXd> v_star =
      VectorX<AutoDiffXd>::LinSpaced(kNumDofs, 0.0, 1.0);
  SapSolver<AutoDiffXd> sap;

  // The case without constraints.
  SapContactProblem<AutoDiffXd> contact_problem_without_constraint(
      kTimeStep, std::move(A), v_star);
  // Use NaN as the guess as it shouldn't be used at all in the early exit where
  // there's no constraint.
  const VectorX<AutoDiffXd> v_guess =
      VectorX<AutoDiffXd>::Constant(kNumDofs, NAN);
  SapSolverResults<AutoDiffXd> result;
  const SapSolverStatus status =
      sap.SolveWithGuess(contact_problem_without_constraint, v_guess, &result);
  EXPECT_EQ(status, SapSolverStatus::kSuccess);
  EXPECT_EQ(result.v, v_star);
  EXPECT_EQ(result.j, VectorX<AutoDiffXd>::Zero(kNumDofs));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
