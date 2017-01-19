#include <memory>

#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/multibody_tree/math/small_vectors.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra.h"

#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

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
  Matrix3d cross_u = CrossProductMatrix(u);

  // Verify that multiplying [u] * v gives the same result as u x v.
  EXPECT_TRUE(uxv.isApprox(cross_u * v));
}

GTEST_TEST(SmallVectors, SpatialVector) {
  Vector3d w(3, 2, -1);
  Vector3d v(1, 2, 3);

  // Spatial velocity of frame Y measured and expressed in X.
  SpatialVector<double> V_XY(w, v);

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