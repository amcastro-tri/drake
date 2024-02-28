#include "drake/math/diffobj/diff_matrix3.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/diffobj/dense_derivatives.h"
#include "drake/math/diffobj/optional_derivatives.h"

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

template <typename T>
struct get_template;

template <template <class...> class Y, typename... Args>
struct get_template<Y<Args...>> {
  template <typename... Others>
  using type = Y<Others...>;
};

template <class DerivativesType>
class DiffMatrixTests : public ::testing::Test {};

TYPED_TEST_SUITE_P(DiffMatrixTests);

TYPED_TEST_P(DiffMatrixTests, ConstructionFromAutoDiffAndBack) {
  using DiffMatrix3UnderTest = TypeParam;

  // TODO: Work around how to show this in the test's name on the console.
  fmt::print("Type = {}\n", drake::NiceTypeName::Get<DiffMatrix3UnderTest>());

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
  DiffMatrix3UnderTest dm1 = DiffMatrix3UnderTest::MakeFromAutoDiffXd(m1ad);

  EXPECT_EQ(dm1.num_variables(), num_variables);

  MatrixXd dm1_value = ExtractValue(dm1.ToAutoDiffXd());
  MatrixXd dm1_derivatives = ExtractGradient(dm1.ToAutoDiffXd());
  EXPECT_EQ(dm1_value, m1d);
  EXPECT_EQ(dm1_derivatives, grad1);
}

TYPED_TEST_P(DiffMatrixTests, Multiply) {
  using DiffMatrix3UnderTest = TypeParam;

  // TODO: Work around how to show this in the test's name on the console.
  fmt::print("Type = {}\n", drake::NiceTypeName::Get<DiffMatrix3UnderTest>());

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
  DiffMatrix3UnderTest dm1 = DiffMatrix3UnderTest::MakeFromAutoDiffXd(m1ad);
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
  DiffMatrix3UnderTest dm2 = DiffMatrix3UnderTest::MakeFromAutoDiffXd(m2ad);
  EXPECT_EQ(dm2.num_variables(), num_variables);

  // Result.
  Matrix3<AutoDiffXd> mad = m1ad * m2ad;
  DiffMatrix3UnderTest dm = dm1 * dm2;
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

REGISTER_TYPED_TEST_SUITE_P(DiffMatrixTests, ConstructionFromAutoDiffAndBack,
                            Multiply);
typedef ::testing::Types<Matrix3WithDenseDerivatives,
                         Matrix3WithOptionalDerivatives>
    DerivativeTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, DiffMatrixTests, DerivativeTypes);

}  // namespace
}  // namespace internal
}  // namespace diffobj
}  // namespace math
}  // namespace drake