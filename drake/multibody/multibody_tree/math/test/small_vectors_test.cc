#include <memory>

#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/multibody_tree/math/small_vectors.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra_old.h"

#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {

typedef Vector2<double> Vec2d;
typedef Vector3<double> Vec3d;

namespace multibody {
namespace math {
namespace {

using Eigen::Vector3d;
using Eigen::Matrix3d;

GTEST_TEST(SmallVectors, CrossProductMatrix) {
  Vector3d v(1, 2, 4.2);
  Vector3d u(-1.5, 2.2, -2.0);

  // Reference cross product.
  Vector3d uxv = u.cross(v);
  Matrix3d Cu = CrossProductMatrix(u);

  // Verify that multiplying [u] * v gives the same result as u x v.
  EXPECT_TRUE(uxv.isApprox(Cu * v));

  // Checks is skew-symmetric.
  bool is_skew_symmeric =
      (Cu + Cu.transpose()).isZero(Eigen::NumTraits<double>::epsilon());
  EXPECT_TRUE(is_skew_symmeric);
}

GTEST_TEST(SmallVectors, CrossProductMatrixSquared) {
  Vector3d v(1, 2, 4.2);
  Vector3d u(-1.5, 2.2, -2.0);

  // Reference triple vector product.
  Vector3d uxuxv = u.cross(u.cross(v));
  Matrix3d Su = CrossProductMatrixSquared(u);

  PRINT_VARn(Su);
  PRINT_VARn(uxuxv.transpose());
  PRINT_VARn((Su*v).transpose());

  // Checks is symmetric.
  bool is_symmetric =
      IsMatrix3Symmetric(Su, Eigen::NumTraits<double>::epsilon());
  EXPECT_TRUE(is_symmetric);

  // Verify that u x (u x v) = S(u) * v.
  EXPECT_TRUE(uxuxv.isApprox(Su * v));
}

GTEST_TEST(SmallVectors, SpatialVector) {
  Vector3d w(3, 2, -1);
  Vector3d v(1, 2, 3);

  // Spatial velocity of frame Y measured and expressed in X.
  GeneralSpatialVector<double> V_XY(w, v);

  EXPECT_TRUE(V_XY.angular().isApprox(w));
  EXPECT_TRUE(V_XY.linear().isApprox(v));

  // Mutable accessors.
  V_XY.angular() = v;
  V_XY.linear() = w;
  EXPECT_TRUE(V_XY.angular().isApprox(v));
  EXPECT_TRUE(V_XY.linear().isApprox(w));
}

}
}  // math
}  // multibody
}  // drake