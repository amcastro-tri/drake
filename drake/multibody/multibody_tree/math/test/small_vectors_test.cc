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

GTEST_TEST(SmallVectors, CrossProduct) {
  Vec3d v3(1, 2, 0);  // An in-plane vector.
  Vec2d v2(1, 2);     // It's 2D representation.
  Vec3d u3(-1.5, 2.2, 0); // Another in-plane vector.
  Vec2d u2(-1.5, 2.2);    // And it's 2D representation.
  // Spin vectors (think of angular velocity, angular momentum or torque).
  VectorSpace<Vec3d>::Spin w3(0, 0, 3);  // A 3D angular quantity.
  VectorSpace<Vec2d>::Spin w2(3);        // And it's 2D counterpart.

  // Verifies the cross product between two 3D vectors.
  // V3 x V3 ∈ V3
  auto w3xv3 = cross(w3, v3);
  auto w3xv3_eigen = w3.cross(v3);
  EXPECT_NEAR(w3xv3.x(), w3xv3_eigen.x(), Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(w3xv3.y(), w3xv3_eigen.y(), Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(w3xv3.z(), w3xv3_eigen.z(), Eigen::NumTraits<double>::epsilon());

  // Same cross product but between a spin (a vector perpendicular to the 2D
  // plane) and a 2D vector.
  // V1 x V2 ∈ V2
  auto w2xv2 = cross(w2, v2);
  EXPECT_NEAR(w3xv3.x(), w2xv2.x(), Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(w3xv3.y(), w2xv2.y(), Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(w3xv3.z(),       0.0, Eigen::NumTraits<double>::epsilon());

  // Same as above but revert order of arguments.
  // w2 x v2 = -(v2 x w2)
  auto v2xw2 = cross(v2, w2);
  EXPECT_NEAR(v2xw2.x(), -w2xv2.x(), Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(v2xw2.y(), -w2xv2.y(), Eigen::NumTraits<double>::epsilon());

  // Multiplication of two 2D vectors results in an out of plane spin.
  // V2 x V2 ∈ V1
  auto u3xv3 = cross(u3, v3);
  auto u2xv2 = cross(u2, v2);
  EXPECT_NEAR(u3xv3.x(), 0.0, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(u3xv3.y(), 0.0, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(u2xv2.x(), u3xv3.z(), Eigen::NumTraits<double>::epsilon());

  // V1 x V1 = 0.
  auto s = cross(w2, u2xv2) + 1.0;
  EXPECT_NEAR(s.x(), 1.0, Eigen::NumTraits<double>::epsilon());
}

// Tests that Drake's cross product implementation can only take a Vector3<T>
// but not Eigen blocks or general expressions. Therefore .eval() must be called
// on an expression.
// A possible optimization would be to template drake::math::cross() on
// MatrixBase.
GTEST_TEST(SmallVectors, CrossProductWithBlocks) {
  Vec3d u(-1, 3.14, 3);
  Vector6<double> v;
  v << 1, 2, 3, 4, 5, 6;
  auto uxv_eigen = u.cross(v.segment<3>(0));
  auto uxv = cross(u, v.segment<3>(0).eval());
  EXPECT_TRUE(uxv.isApprox(uxv_eigen));
}

GTEST_TEST(SmallVectors, CrossProductMatrix) {
  // Test in 3D.
  {
    typedef Vector3<double> Vec;
    typedef VectorSpace<Vec>::Spin Spin;
    Vec v(1, 2, 4.2);
    Vec u(-1.5, 2.2, -2.0);

    // Reference cross product.
    Spin uxv = u.cross(v);
    auto cross_u = CrossProductMatrix(u);

    // Verify that multiplying [u] * v gives the same result as u x v.
    EXPECT_TRUE(uxv.isApprox(cross_u * v));
  }

  // Test in 2D.
  {
    typedef Vector2<double> Vec;
    typedef VectorSpace<Vec>::Spin Spin;
    Vec v(1, 2);
    Vec u(-1.5, 2.2);

    // Reference cross product.
    Spin uxv = cross(u, v);
    auto cross_u = CrossProductMatrix(u);

    // Verify that multiplying [u] * v gives the same result as u x v.
    EXPECT_TRUE(uxv.isApprox(cross_u * v));
  }
}

GTEST_TEST(SmallVectors, SpatialVector) {
  // Tests in 3D.
  {
    typedef Vector3<double> Vec;
    typedef VectorSpace<Vec>::Spin Spin;

    Spin w(3, 2, -1);
    Vec v(1, 2, 3);

    // Spatial velocity of frame Y measured and expressed in X.
    SpatialVector<Vec> V_XY(w, v);

    EXPECT_TRUE(V_XY.angular().isApprox(w));
    EXPECT_TRUE(V_XY.linear().isApprox(v));

    // Mutable accessors.
    V_XY.angular() = v;
    V_XY.linear() = w;
    EXPECT_TRUE(V_XY.angular().isApprox(v));
    EXPECT_TRUE(V_XY.linear().isApprox(w));
  }

  // Tests in 2D.
  {
    typedef Vector2<double> Vec;
    typedef VectorSpace<Vec>::Spin Spin;

    Spin w(3);
    Vec v(1, 2);

    // Spatial velocity of frame Y measured and expressed in X.
    SpatialVector<Vec> V_XY(w, v);

    EXPECT_TRUE(V_XY.angular().isApprox(w));
    EXPECT_TRUE(V_XY.linear().isApprox(v));

    // Mutable accessors.
    w << -1;
    v << -2, 8;
    V_XY.angular() = w;
    V_XY.linear().transpose() = v.transpose();
    EXPECT_TRUE(V_XY.angular().isApprox(w));
    EXPECT_TRUE(V_XY.linear().isApprox(v));

    // The following line tests the non-aligned implementation of
    // SpatialVector::linear() for the 2D specialization.
    // For this case *only*, it was found that the print statement below
    // causes a segfault when the return type is aligned. No similar problem
    // was observed in 3D.
    // No crash is a successful test.
    // While at it, check the correct value of the string expression.
    std::stringstream stream;
    stream << V_XY.linear().transpose();
    EXPECT_EQ(stream.str(), "-2  8");
  }
}

}
}  // math
}  // multibody
}  // drake