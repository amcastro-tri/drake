#include "drake/multibody/contact_solvers/contact_problem.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

// clang-format off
const Matrix3d S22 =
    (Eigen::Matrix3d() << 2, 1,
                          1, 2).finished();
const Matrix3d S33 =
    (Eigen::Matrix3d() << 4, 1, 2,
                          1, 5, 3,
                          2, 3, 6).finished();
const Matrix3d S44 =
    (Eigen::Matrix3d() << 7, 1, 2, 3,
                          1, 8, 4, 5,
                          2, 4, 9, 6,
                          3, 5, 6, 10).finished();
// clang-format on

// clang-format off
const Matrix3d J32 =
    (Eigen::Matrix3d() << 2, 1,
                          1, 2,
                          1, 2).finished();
const Matrix3d J33 =
    (Eigen::Matrix3d() << 4, 1, 2,
                          1, 5, 3,
                          2, 3, 6).finished();
const Matrix3d J34 =
    (Eigen::Matrix3d() << 7, 1, 2, 3,
                          1, 8, 4, 5,
                          2, 4, 9, 6).finished();
// clang-format on


GTEST_TEST(ContactProblem, Construction) {
  std::vector<MatrixXd> A{S22, S33, S44};
  VectorXd v_star = VectorXd::LinSpaced(9, 1.0, 9.0);
  SapContactProblem<double> problem(std::move(A), std::move(v_star));
  EXPECT_EQ(problem.num_cliques(), 3);
  EXPECT_EQ(problem.num_constraints(), 0);
  EXPECT_EQ(problem.num_velocities(), 9);

  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(0, 1, J32, J33, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(1, 2, J33, J34, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(2, J34, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(0, 2, J32, J34, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(0, 2, J32, J34, 1.0));
  EXPECT_EQ(problem.num_constraints(), 5);

  const ContactProblemGraph graph = problem.MakeGraph();
  EXPECT_EQ(graph.num_cliques(), 3);  
  EXPECT_EQ(graph.num_edges(), 4);
  EXPECT_EQ(graph.num_constraints(), 5);


}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake