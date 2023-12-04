#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sap/sap_hunt_crossley.h"
#include "drake/multibody/contact_solvers/sap/validate_constraint_gradients.h"
#include "drake/solvers/constraint.h"

using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace kcov339_avoidance_magic {
namespace {

// These Jacobian matrices have arbitrary values for testing. We specify the
// size of the matrix in the name, e.g. J32 is of size 3x2.
// clang-format off
const MatrixXd J32 =
    (MatrixXd(3, 2) << 2, 1,
                       1, 2,
                       1, 2).finished();

const MatrixXd J34 =
    (MatrixXd(3, 4) << 7, 1, 2, 3,
                       1, 8, 4, 5,
                       2, 4, 9, 6).finished();
// clang-format on

template <typename T = double>
typename SapHuntCrossley<T>::Parameters MakeArbitraryParameters() {
  const T mu = 0.5;
  const T stiffness = 1.0e5;
  const T dissipation = 0.01;
  const double beta = 0.01;
  const double vs = 1.0e-4;
  return typename SapHuntCrossley<T>::Parameters{mu, stiffness, dissipation,
                                                 beta, vs};
}

GTEST_TEST(SapHuntCrossley, SingleCliqueConstraint) {
  const int clique = 12;
  const double fe0 = 150.0;
  SapConstraintJacobian<double> J(clique, J32);
  const SapHuntCrossley<double>::Parameters parameters =
      MakeArbitraryParameters();
  SapHuntCrossley<double> c(fe0, std::move(J), parameters);

  // EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), clique);
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
  EXPECT_EQ(c.mu(), parameters.mu);
  EXPECT_EQ(c.parameters().mu, parameters.mu);
  EXPECT_EQ(c.parameters().stiffness, parameters.stiffness);
  EXPECT_EQ(c.parameters().dissipation, parameters.dissipation);
  EXPECT_EQ(c.parameters().beta, parameters.beta);
  EXPECT_EQ(c.parameters().vs, parameters.vs);
}

GTEST_TEST(SapHuntCrossley, TwoCliquesConstraint) {
  const int clique0 = 12;
  const int clique1 = 13;
  const double fe0 = 150.0;
  SapConstraintJacobian<double> J(clique0, J32, clique1, J34);
  const SapHuntCrossley<double>::Parameters parameters =
      MakeArbitraryParameters();
  SapHuntCrossley<double> c(fe0, std::move(J), parameters);

  // EXPECT_EQ(c.num_objects(), 2);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), clique0);
  EXPECT_EQ(c.second_clique(), clique1);
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_EQ(c.second_clique_jacobian().MakeDenseMatrix(), J34);
  EXPECT_EQ(c.mu(), parameters.mu);
  EXPECT_EQ(c.parameters().mu, parameters.mu);
  EXPECT_EQ(c.parameters().stiffness, parameters.stiffness);
  EXPECT_EQ(c.parameters().dissipation, parameters.dissipation);
  EXPECT_EQ(c.parameters().beta, parameters.beta);
  EXPECT_EQ(c.parameters().vs, parameters.vs);
}

// This method validates analytical gradients implemented by
// SapHuntCrossley using automatic differentiation.
void ValidateProjection(const SapHuntCrossley<double>::Parameters& p,
                        const Vector3d& vc) {
  // Instantiate constraint on AutoDiffXd for automatic differentiation.
  SapHuntCrossley<AutoDiffXd>::Parameters p_ad{p.mu, p.stiffness, p.dissipation,
                                               p.beta, p.vs};

  // Arbitrary set of parameters.
  const int clique = 0;
  const AutoDiffXd fe0 = 15.0;
  const double infty = std::numeric_limits<double>::infinity();
  SapConstraintJacobian<AutoDiffXd> J(clique, Matrix3<double>::Constant(infty));

  const SapHuntCrossley<AutoDiffXd> c(fe0, std::move(J), std::move(p_ad));

  // Verify cost gradients using AutoDiffXd.
  ValidateConstraintGradients(c, vc);
}

// Stiction.
GTEST_TEST(SapHuntCrossley, RegionI) {
  // An arbitrary set of parameters.
  SapHuntCrossley<double>::Parameters p;
  p.mu = 0.5;
  p.stiffness = 3.0e5;
  p.dissipation = 15.0;

  // Below we use an arbitrary set of values so that vc leads to stiction.
  {
    const Vector3d vc(1e-5, 0, -0.1);
    ValidateProjection(p, vc);
  }
  {
    const Vector3d vc(1e-5, 2e-5, -0.1);
    ValidateProjection(p, vc);
  }
  {
    const Vector3d vc(-1e-5, 2e-5, -0.05);
    ValidateProjection(p, vc);
  }
}

// Sliding.
GTEST_TEST(SapHuntCrossley, RegionII) {
  // An arbitrary set of parameters.
  SapHuntCrossley<double>::Parameters p;
  p.mu = 0.5;
  p.stiffness = 3.0e5;
  p.dissipation = 15.0;

  // Below we use an arbitrary set of values so that vc leads to sliding.
  {
    const Vector3d vc(0.1, 0, -0.1);
    ValidateProjection(p, vc);
  }
  {
    const Vector3d vc(0.1, -0.2, -0.1);
    ValidateProjection(p, vc);
  }
  {
    const Vector3d vc(0.1, 0.05, 0.0);
    ValidateProjection(p, vc);
  }
}

// No contact
GTEST_TEST(SapHuntCrossley, RegionIII) {
  // An arbitrary set of parameters.
  SapHuntCrossley<double>::Parameters p;
  p.mu = 0.5;
  p.stiffness = 3.0e5;
  p.dissipation = 15.0;

  // Below we use an arbitrary set of values so that vc leads to no-contact.
  {
    const Vector3d vc(0.1, 0, 0.1);
    ValidateProjection(p, vc);
  }
  {
    const Vector3d vc(1.0e-4, -2.0e-3, 0.01);
    ValidateProjection(p, vc);
  }
  {
    const Vector3d vc(-0.1, 0.1, 0.2);
    ValidateProjection(p, vc);
  }
}

GTEST_TEST(SapHuntCrossley, RegionIZeroStiffnessAndZeroDissipation) {
  // An arbitrary set of parameters.
  SapHuntCrossley<double>::Parameters p;
  p.mu = 0.5;
  p.stiffness = 1.0e-14;
  p.dissipation = 0.0;

  // Below we use an arbitrary set of values so that vc leads to stiction.
  {
    const Vector3d vc(1e-5, 0, -0.1);
    ValidateProjection(p, vc);
  }
  {
    const Vector3d vc(1e-5, 2e-5, -0.1);
    ValidateProjection(p, vc);
  }
  {
    const Vector3d vc(-1e-5, 2e-5, -0.05);
    ValidateProjection(p, vc);
  }
}

GTEST_TEST(SapHuntCrossley, RegionIIZeroStiffnessAndZeroDissipation) {
  // An arbitrary set of parameters.
  SapHuntCrossley<double>::Parameters p;
  p.mu = 0.5;
  p.stiffness = 1.0e-14;
  p.dissipation = 0.0;

  // Below we use an arbitrary set of values so that vc leads to sliding.
  {
    const Vector3d vc(0.1, 0, -0.1);
    ValidateProjection(p, vc);
  }
  {
    const Vector3d vc(0.1, -0.2, -0.1);
    ValidateProjection(p, vc);
  }
  {
    const Vector3d vc(0.1, 0.05, 0.0);
    ValidateProjection(p, vc);
  }
}

GTEST_TEST(SapHuntCrossley, SingleCliqueConstraintClone) {
  const int clique = 12;
  const double fe0 = 150.0;
  SapConstraintJacobian<double> J(clique, J32);
  const SapHuntCrossley<double>::Parameters parameters =
      MakeArbitraryParameters();
  SapHuntCrossley<double> c(fe0, std::move(J), parameters);

  // N.B. Here we dynamic cast to the derived type so that we can test that the
  // clone is a deep-copy of the original constraint.
  auto clone = dynamic_pointer_cast<SapHuntCrossley<double>>(c.Clone());
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->num_constraint_equations(), 3);
  EXPECT_EQ(clone->num_cliques(), 1);
  EXPECT_EQ(clone->first_clique(), clique);
  EXPECT_THROW(clone->second_clique(), std::exception);
  EXPECT_EQ(clone->first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_THROW(clone->second_clique_jacobian(), std::exception);
  EXPECT_EQ(clone->mu(), parameters.mu);
  EXPECT_EQ(clone->parameters().mu, parameters.mu);
  EXPECT_EQ(clone->parameters().stiffness, parameters.stiffness);
  EXPECT_EQ(clone->parameters().dissipation, parameters.dissipation);
  EXPECT_EQ(clone->parameters().beta, parameters.beta);
  EXPECT_EQ(clone->parameters().vs, parameters.vs);
}

GTEST_TEST(SapHuntCrossley, TwoCliquesConstraintClone) {
  const int clique0 = 12;
  const int clique1 = 13;
  const double fe0 = 150.0;
  SapConstraintJacobian<double> J(clique0, J32, clique1, J34);
  const SapHuntCrossley<double>::Parameters parameters =
      MakeArbitraryParameters();
  SapHuntCrossley<double> c(fe0, std::move(J), parameters);

  auto clone = dynamic_pointer_cast<SapHuntCrossley<double>>(c.Clone());
  EXPECT_EQ(clone->num_constraint_equations(), 3);
  EXPECT_EQ(clone->num_cliques(), 2);
  EXPECT_EQ(clone->first_clique(), clique0);
  EXPECT_EQ(clone->second_clique(), clique1);
  EXPECT_EQ(clone->first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_EQ(clone->second_clique_jacobian().MakeDenseMatrix(), J34);
  EXPECT_EQ(clone->mu(), parameters.mu);
  EXPECT_EQ(clone->parameters().mu, parameters.mu);
  EXPECT_EQ(clone->parameters().stiffness, parameters.stiffness);
  EXPECT_EQ(clone->parameters().dissipation, parameters.dissipation);
  EXPECT_EQ(clone->parameters().beta, parameters.beta);
  EXPECT_EQ(clone->parameters().vs, parameters.vs);
}

}  // namespace
}  // namespace kcov339_avoidance_magic
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
