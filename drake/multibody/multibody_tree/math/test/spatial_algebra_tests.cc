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

GTEST_TEST(SpatialAlgebra, SpatialVelocityShift) {
  typedef Vector3<double> Vec;
  typedef VectorSpace<Vec>::Spin Spin;

  // Linear velocity of frame B measured and expressed in frame A.
  Vec v_AB(1, 2, 0);

  // Angular velocity of frame B measured and expressed in frame A.
  Spin w_AB(0, 0, 3);

  // Spatial velocity of frame B with respect to A and expressed in A.
  SpatialVector<Vec> V_AB(w_AB, v_AB);

  // A vector from Bo to Qo, expressed in A.
  Vec r_BQ_A(2, -2, 0);

  SpatialVector<Vec> V_AQ =
      ShiftOperator<Vec>::ShiftSpatialVelocity(V_AB, r_BQ_A);

  PRINT_VAR(V_AQ);

  ShiftOperator<Vec> phi_XY(r_BQ_A);

  PRINT_VAR(phi_XY.transpose() * V_AB);

}

}
}  // math
}  // multibody
}  // drake