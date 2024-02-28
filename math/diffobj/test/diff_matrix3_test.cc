#include "drake/math/diffobj/diff_matrix3.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::AutoDiffScalar;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

#include <iostream>
#define PRINT_VAR(a) std::cout << #a << a << std::endl;
#define PRINT_VARn(a) std::cout << #a "\n" << a << std::endl;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

namespace drake {
namespace math {
namespace diffobj {
namespace internal {
namespace {

GTEST_TEST(DiffMatrix3, ConstructionFromAutoDiffAndBack) {
  const int num_variables = 12;
  // clang-format off
  Matrix3d m1d = (Matrix3d() << 
    1, 2, 3,
    4, 5, 6,
    7, 8, 9).finished();
  // clang-format on
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
  Matrix3WithDenseDerivatives dm1 =
      Matrix3WithDenseDerivatives::MakeFromAutoDiffXd(m1ad);

  EXPECT_EQ(dm1.num_variables(), num_variables);

  MatrixXd dm1_value = ExtractValue(dm1.ToAutoDiffXd());
  MatrixXd dm1_derivatives = ExtractGradient(dm1.ToAutoDiffXd());
  EXPECT_EQ(dm1_value, m1d);
  EXPECT_EQ(dm1_derivatives, grad1);
}

GTEST_TEST(DiffMatrix3, Multiply) {
  const int num_variables = 12;
  // Rotation 1.
  // clang-format off
  Matrix3d m1d = (Matrix3d() << 
    1, 2, 3,
    4, 5, 6,
    7, 8, 9).finished();
  // clang-format on
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
  Matrix3WithDenseDerivatives dm1 =
      Matrix3WithDenseDerivatives::MakeFromAutoDiffXd(m1ad);
  EXPECT_EQ(dm1.num_variables(), num_variables);

  // Rotation 2.
  // clang-format off
  Matrix3d m2d = (Matrix3d() << 
    1, 2, 3,
    4, 5, 6,
    7, 8, 9).finished().transpose();
  // clang-format on
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
  Matrix3WithDenseDerivatives dm2 =
      Matrix3WithDenseDerivatives::MakeFromAutoDiffXd(m2ad);
  EXPECT_EQ(dm2.num_variables(), num_variables);

  // Result.
  Matrix3<AutoDiffXd> mad = m1ad * m2ad;
  Matrix3WithDenseDerivatives dm = dm1 * dm2;
  EXPECT_EQ(dm.num_variables(), num_variables);

  EXPECT_TRUE(CompareMatrices(ExtractValue(dm.ToAutoDiffXd()),
                              ExtractValue(mad), kEpsilon,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(ExtractGradient(dm.ToAutoDiffXd()),
                              ExtractGradient(mad), kEpsilon,
                              MatrixCompareType::relative));

  fmt::print("dm.gradient():\n{}\n",
             fmt_eigen(ExtractGradient(dm.ToAutoDiffXd())));
}

}  // namespace
}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake