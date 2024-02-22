#include "drake/math/rotation_matrix_with_derivatives.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic/expression.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::AutoDiffScalar;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

namespace drake {
namespace math {
namespace internal {
namespace {

GTEST_TEST(RotationWithDerivatives, Construction) {
  {
    RotationMatrixWithDerivatives R;
    EXPECT_EQ(R.num_derivatives(), 0);
  }

  // N.B. This is not a valid rotation matrix. Irrelevan for these tests.
  Matrix3d m;
  // clang-format off
  m << 1, 2, 3,
       4, 5, 6,
       7, 8, 9;
  // clang-format on

  {
    RotationMatrixWithDerivatives R(m);
    EXPECT_EQ(R.num_derivatives(), 0);
    EXPECT_EQ(R.value(), m);
  }

  std::vector<Matrix3d> d(5);
  for (int i = 0; i < 5; ++i) {
    d[i] = 0.1 * m + Matrix3d::Constant(0.2 * i);
  }

  {
    RotationMatrixWithDerivatives R(m, d);
    EXPECT_EQ(R.num_derivatives(), 5);
    EXPECT_EQ(R.value(), m);
    EXPECT_EQ(R.derivatives(), d);
  }
}

GTEST_TEST(RotationWithDerivatives, Multiply) {
  const int kNumDerivatives = 5;
  Matrix3d v1;
  // clang-format off
  v1 << 1, 2, 3,
        4, 5, 6,
        7, 8, 9;
  // clang-format on
  std::vector<Matrix3d> d1(kNumDerivatives);
  for (int i = 0; i < kNumDerivatives; ++i) {
    d1[i] = 0.1 * v1 + Matrix3d::Constant(0.2 * i);
  }

  Matrix3d v2 = 0.3 * v1;
  std::vector<Matrix3d> d2(kNumDerivatives);
  for (int i = 0; i < kNumDerivatives; ++i) {
    d2[i] = 0.1 * v2 + Matrix3d::Constant(0.2 * i);
  }

  const Matrix3d expected_v = v1 * v2;
  std::vector<Matrix3d> expected_d(kNumDerivatives);
  for (int i = 0; i < kNumDerivatives; ++i) {
    expected_d[i] = d1[i] * v2 + v1 * d2[i];
  }

  const RotationMatrixWithDerivatives R1(v1, d1);
  const RotationMatrixWithDerivatives R2(v2, d2);

  const RotationMatrixWithDerivatives R = R1 * R2;
  EXPECT_EQ(R.num_derivatives(), kNumDerivatives);
  EXPECT_EQ(R.value(), expected_v);
  EXPECT_EQ(R.derivatives(), expected_d);
}

}  // namespace
}  // namespace internal
}  // namespace math
}  // namespace drake