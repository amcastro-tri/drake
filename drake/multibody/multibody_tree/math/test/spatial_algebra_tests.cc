#include <memory>

#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/multibody_tree/math/small_vectors.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::Matrix;
using Eigen::Vector3d;

GTEST_TEST(SpatialAlgebra, SpatialVelocityShift) {
  // Linear velocity of frame B measured and expressed in frame A.
  Vector3d v_AB(1, 2, 0);

  // Angular velocity of frame B measured and expressed in frame A.
  Vector3d w_AB(0, 0, 3);

  // Spatial velocity of frame B with respect to A and expressed in A.
  SpatialVector<double> V_AB(w_AB, v_AB);

  // A shift operator representing the rigid body transformation from frame B
  // to frame Q, expressed in A.
  ShiftOperator<double> phi_BQ_A({2, -2, 0});

  SpatialVector<double> V_AQ = phi_BQ_A.transpose() * V_AB;
  SpatialVector<double> expected_V_AQ(w_AB, {7, 8, 0});

  EXPECT_TRUE(V_AQ.angular().isApprox(expected_V_AQ.angular()));
  EXPECT_TRUE(V_AQ.linear().isApprox(expected_V_AQ.linear()));
}

GTEST_TEST(SpatialAlgebra, SpatialVelocityJacobianShift) {
  // Linear velocity of frame B measured and expressed in frame A.
  Vector3d v_AB(1, 2, 0);

  // Angular velocity of frame B measured and expressed in frame A.
  Vector3d w_AB(0, 0, 3);

  // Spatial velocity of frame B with respect to A and expressed in A.
  SpatialVector<double> V_AB(w_AB, v_AB);

  const int ncols = 3;
  SpatialVelocityJacobian<double, ncols> J_AB;

  J_AB.col(0) =      V_AB;
  J_AB.col(1) = 2. * V_AB;
  J_AB.col(2) = 3. * V_AB;

  // A shift operator representing the rigid body transformation from frame B
  // to frame Q, expressed in A.
  ShiftOperator<double> phi_BQ_A({2, -2, 0});

  SpatialVelocityJacobian<double, ncols> J_AQ = phi_BQ_A.transpose() * J_AB;
  SpatialVelocityJacobian<double, ncols> expected_J_AQ;
  expected_J_AQ.col(0) =      SpatialVector<double>(w_AB, {7, 8, 0});
  expected_J_AQ.col(1) = 2. * SpatialVector<double>(w_AB, {7, 8, 0});
  expected_J_AQ.col(2) = 3. * SpatialVector<double>(w_AB, {7, 8, 0});

  EXPECT_TRUE(J_AQ.col(0).angular().isApprox(expected_J_AQ.col(0).angular()));
  EXPECT_TRUE(J_AQ.col(1).angular().isApprox(expected_J_AQ.col(1).angular()));
  EXPECT_TRUE(J_AQ.col(2).angular().isApprox(expected_J_AQ.col(2).angular()));

  EXPECT_TRUE(J_AQ.col(0).linear().isApprox(expected_J_AQ.col(0).linear()));
  EXPECT_TRUE(J_AQ.col(1).linear().isApprox(expected_J_AQ.col(1).linear()));
  EXPECT_TRUE(J_AQ.col(2).linear().isApprox(expected_J_AQ.col(2).linear()));
}

GTEST_TEST(SpatialAlgebra, DynamicSizeJacobian) {
  SpatialVelocityJacobian<double, Eigen::Dynamic> J1;
  EXPECT_EQ(J1.rows(), 6);
  EXPECT_EQ(J1.cols(), 0);

  J1.resize(4);
  EXPECT_EQ(J1.rows(), 6);
  EXPECT_EQ(J1.cols(), 4);

  SpatialVelocityJacobian<double, Eigen::Dynamic> J2(3);
  EXPECT_EQ(J2.rows(), 6);
  EXPECT_EQ(J2.cols(), 3);

  J2.resize(4);
  EXPECT_EQ(J2.rows(), 6);
  EXPECT_EQ(J2.cols(), 4);

  // For SpatialVelocityJacobianUpTo6 the maximum number of colums must be 6.
  EXPECT_EQ(SpatialVelocityJacobianUpTo6<double>::kMaxCols, 6);
  // For any other know at compile time Jacobian size the maximum number of
  // columns must match this known size.
  // Note: EXPECT_EQ is a macro and therefore gets confused with angle brackets
  // and commas in between. Therefore we use an extra set of parentheses.
  EXPECT_EQ((SpatialVelocityJacobian<double, 1>::kMaxCols), 1);
  EXPECT_EQ((SpatialVelocityJacobian<double, 3>::kMaxCols), 3);
  EXPECT_EQ((SpatialVelocityJacobian<double, 5>::kMaxCols), 5);

  // Even though the maximum number of columns is six we still can resize
  // the number of columns to any other number lower or equal than six.
  SpatialVelocityJacobianUpTo6<double> J3;
  EXPECT_EQ(J3.rows(), 6);
  EXPECT_EQ(J3.cols(), 0);
  J3.resize(4);
  EXPECT_EQ(J3.rows(), 6);
  EXPECT_EQ(J3.cols(), 4);
}

GTEST_TEST(SpatialAlgebra, SpatialVelocityJacobianTranspose) {
  // Linear velocity of frame B measured and expressed in frame A.
  Vector3d v_AB(1, 2, 0);

  // Angular velocity of frame B measured and expressed in frame A.
  Vector3d w_AB(0, 0, 3);

  // Spatial velocity of frame B with respect to A and expressed in A.
  SpatialVector<double> V_AB(w_AB, v_AB);

  const int ncols = 3;
  SpatialVelocityJacobian<double, ncols> J_AB;

  J_AB.col(0) =      V_AB;
  J_AB.col(1) = 2. * V_AB;
  J_AB.col(2) = 3. * V_AB;

  // Convert J_AB into an Eigen matrix.
  Matrix<double, 6, 3> J_AB_matrix = J_AB.get_coeffs();

  // Get the transpose of J_AB and convert it to an Eigen matrix.
  Matrix<double, 3, 6> Jt_AB_matrix = J_AB.transpose().get_coeffs();

  EXPECT_TRUE(J_AB_matrix.isApprox(Jt_AB_matrix.transpose(),
                                   Eigen::NumTraits<double>::epsilon()));
}

}
}  // math
}  // multibody
}  // drake