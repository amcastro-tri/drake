#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/roll_pitch_yaw.h"

using drake::math::RotationMatrixd;
using drake::math::RollPitchYawd;
using Eigen::AutoDiffScalar;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

#include <iostream>
#define PRINT_VAR(a) std::cout << #a << a << std::endl;
#define PRINT_VARn(a) std::cout << #a"\n" << a << std::endl;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

namespace drake {
namespace math {
namespace {

GTEST_TEST(RotationWithDerivatives, Construction) {
  const int num_variables = 12;  
  // Rotation 1.  
  Matrix3d m1d = RotationMatrixd(RollPitchYawd(Vector3d(1., 2., 3.))).matrix();
  VectorXd grad_col = VectorXd::LinSpaced(9, 1.0, 9.0);
  Eigen::Matrix<double, 9, Eigen::Dynamic> grad1(
      9, num_variables);  // Each partial stored in a column.
  grad1.setZero();
  // Only a few columsn are non-zero.
  grad1.col(0) = grad_col;
  grad1.col(5) = 0.2 * grad_col;
  grad1.col(10) = -0.1 * grad_col;
  Matrix3<AutoDiffXd> m1ad;
  InitializeAutoDiff(m1d, grad1, &m1ad);
  RotationMatrix<AutoDiffXd> R1ad(m1ad);
    
  EXPECT_EQ(R1ad.num_variables(), num_variables);
  EXPECT_EQ(R1ad.num_non_zeros(), 3);

  MatrixXd R1ad_value = ExtractValue(R1ad.matrix());
  MatrixXd R1ad_derivatives = ExtractGradient(R1ad.matrix());
  EXPECT_EQ(R1ad_value, m1d);
  EXPECT_EQ(R1ad_derivatives, grad1);
}

GTEST_TEST(RotationWithDerivatives, Multiply) {
  const int num_variables = 12;  
  // Rotation 1.  
  Matrix3d m1d = RotationMatrixd(RollPitchYawd(Vector3d(1., 2., 3.))).matrix();
  VectorXd grad_col = VectorXd::LinSpaced(9, 1.0, 9.0);
  Eigen::Matrix<double, 9, Eigen::Dynamic> grad1(
      9, num_variables);  // Each partial stored in a column.
  grad1.setZero();
  // Only a few columsn are non-zero.
  grad1.col(1) = grad_col;
  grad1.col(5) = 0.2 * grad_col;
  grad1.col(10) = -0.1 * grad_col;
  Matrix3<AutoDiffXd> m1ad;
  InitializeAutoDiff(m1d, grad1, &m1ad);
  RotationMatrix<AutoDiffXd> R1ad(m1ad);
  EXPECT_EQ(R1ad.num_variables(), num_variables);
  EXPECT_EQ(R1ad.num_non_zeros(), 3);

  // Rotation 2.  
  Matrix3d m2d = RotationMatrixd(RollPitchYawd(Vector3d(3., 2., 1.))).matrix();
  VectorXd grad2_col = -0.2 * grad_col;
  Eigen::Matrix<double, 9, Eigen::Dynamic> grad2(
      9, num_variables);  // Each partial stored in a column.
  grad2.setZero();
  // Only a few columsn are non-zero.
  grad2.col(0) = grad_col;
  grad2.col(4) = -1.2 * grad_col;
  grad2.col(5) = 0.2 * grad_col;
  grad2.col(10) = -0.1 * grad_col;
  Matrix3<AutoDiffXd> m2ad;
  InitializeAutoDiff(m2d, grad2, &m2ad);
  RotationMatrix<AutoDiffXd> R2ad(m2ad);
  EXPECT_EQ(R2ad.num_variables(), num_variables);
  EXPECT_EQ(R2ad.num_non_zeros(), 4);

  // Result.
  Matrix3<AutoDiffXd> mad = m1ad * m2ad;
  RotationMatrix<AutoDiffXd> Rad = R1ad * R2ad;
  EXPECT_EQ(Rad.num_variables(), num_variables);
  EXPECT_EQ(Rad.num_non_zeros(), 5);

  EXPECT_TRUE(
      CompareMatrices(ExtractValue(Rad.matrix()), ExtractValue(mad), kEpsilon));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(Rad.matrix()),
                              ExtractGradient(mad), 10 * kEpsilon));

  fmt::print("Rad.gradient():\n{}\n", fmt_eigen(ExtractGradient(Rad.matrix())));
}



}  // namespace
}  // namespace math
}  // namespace drake
