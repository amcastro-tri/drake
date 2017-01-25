#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"

#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {
namespace math {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;

GTEST_TEST(RotationalInertia, Symmetry) {
  RotationalInertia<double> I(3.14);
  //PRINT_VARn(I);
  (void)I;
  std::cout << I(0,0) << std::endl;
  std::cout << I(0,2) << std::endl;
  std::cout << I(2,0) << std::endl;
  std::cout << I(1,1) << std::endl;
  std::cout << std::endl;

  //I(0,2) = -1.0;
  I(2,0) = -1.0;
  std::cout << I(0,0) << std::endl;
  std::cout << I(0,2) << std::endl;
  std::cout << I(2,0) << std::endl;
  std::cout << I(1,1) << std::endl;


  PRINT_VARn(I);

  PRINT_VARn(I.get_matrix());

  PRINT_VARn(I.CopyToFullMatrix3());

  Matrix3<double> m = I.get_symmetric_matrix_view();
  PRINT_VARn(m);
  //std::cout << I.get_matrix();
}

GTEST_TEST(RotationalInertia, ReExpressInAnotherFrame) {
  const double radius = 0.1;
  const double length = 1.0;
  // Rod frame R located at the rod's geometric center and oriented along its
  // principal axes.
  // Inertia computed about Ro and expressed in R.
  RotationalInertia<double> I_Ro_R =
      RotationalInertia<double>::SolidRod(radius, length);
  // Momentum about its axis aligned with R's z-axis.
  const double Irr = I_Ro_R(2, 2);
  // Moment of inertia about an axis perpendicular to the rod's axis.
  const double Iperp = I_Ro_R(0, 0);

  PRINT_VARn(I_Ro_R);

  // Re-express on a frame F obtained by rotating R +90 degrees about x.
  Matrix3<double> R_FR =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();
  PRINT_VARn(R_FR);

  RotationalInertia<double> I_Ro_F = I_Ro_R.ReExpress(R_FR);

  PRINT_VARn(I_Ro_F.get_matrix());

  // Now the R's z-axis is oriented along F's y-axis.
  EXPECT_NEAR(I_Ro_F(0, 0), Iperp, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(I_Ro_F(1, 1), Irr, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(I_Ro_F(2, 2), Iperp, Eigen::NumTraits<double>::epsilon());

  PRINT_VARn(I_Ro_F);

  // Check if after transformation this still is a physically valid inertia.
  EXPECT_TRUE(I_Ro_F.IsPhysicallyValid());

}

GTEST_TEST(SpatialInertia, PlusEqualOperator) {
  const double L = 2.0;
  // Rod frame R located at the rod's geometric center and oriented along its
  // principal axes.
  // Inertia computed about Ro and expressed in R.

  // Spatial inertia computed about the origin for a cube with sides of
  // length 2.0 centered at x = 1.0. Expressed in world frame.
  const double mass_right = 1.5;
  SpatialInertia<double> MRightBox_Wo_W(
      mass_right,
      Vector3d::Zero(),
      mass_right * RotationalInertia<double>::SolidCube(L));
  MRightBox_Wo_W.ShiftOriginInPlace(-Vector3d::UnitX());
  // Check if after transformation this still is a physically valid inertia.
  EXPECT_TRUE(MRightBox_Wo_W.IsPhysicallyValid());

  PRINT_VARn(MRightBox_Wo_W);

  // Spatial inertia computed about the origin for a cube with sides of
  // length 2.0 centered at x = -1.0. Expressed in world frame.
  const double mass_left = 0.5;
  SpatialInertia<double> MLeftBox_Wo_W(
      mass_left,
      Vector3d::Zero(),
      mass_left * RotationalInertia<double>::SolidCube(L));
  MLeftBox_Wo_W.ShiftOriginInPlace(Vector3d::UnitX());
  // Check if after transformation this still is a physically valid inertia.
  EXPECT_TRUE(MLeftBox_Wo_W.IsPhysicallyValid());

  PRINT_VARn(MLeftBox_Wo_W);

  // Spatial inertia of a prism with a squared transverse area of size
  // 2.0 x 2.0 and length 4.0.
  // This is computed by adding the above spatial inertias.
  // Notice that the origina and the expressed-in frame is the same as in the
  // two individual components.
  SpatialInertia<double> MPrism_Wo_W(MLeftBox_Wo_W);
  MPrism_Wo_W += MRightBox_Wo_W;
  EXPECT_TRUE(MPrism_Wo_W.IsPhysicallyValid());

  PRINT_VARn(MPrism_Wo_W);

  // Check that the compound inertia corresponds to that of a larger box of
  // length 4.0.
  const double mass = mass_left + mass_right;
  const Vector3d com(
      (mass_left * MLeftBox_Wo_W.get_com() +
       mass_right * MRightBox_Wo_W.get_com()) / mass);
  SpatialInertia<double> MExpected_Wo_W(
      mass,
      com,
      mass * RotationalInertia<double>::SolidBox(2 * L, L, L));
  // Check if after transformation this still is a physically valid inertia.
  EXPECT_TRUE(MExpected_Wo_W.IsPhysicallyValid());

  PRINT_VARn(MExpected_Wo_W);

  EXPECT_TRUE(MPrism_Wo_W.IsApprox(MExpected_Wo_W));
}

GTEST_TEST(SpatialInertia, ReExpress) {
  // Spatial inertia for a cube of unit mass computed about its center of mass
  // and expressed in its principal axes frame.
  const double Lx = 0.2, Ly = 1.0, Lz = 0.5;  // Box's lengths.
  SpatialInertia<double> M_Bo_B(
      1.0, Vector3d::UnitX(), RotationalInertia<double>::SolidBox(Lx, Ly, Lz));

  // Replace this pring by the respective getters and check values.
  PRINT_VARn(M_Bo_B);

  // Place B rotated +90 degrees about W's x-axis.
  Matrix3<double> R_WB =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();
  PRINT_VARn(R_WB);

  SpatialInertia<double> M_Bo_W = M_Bo_B.ReExpress(R_WB);

  // Checks for physically correct spatial inertia.
  EXPECT_TRUE(M_Bo_W.IsPhysicallyValid());

  // Replace this print by the respective checks of the new computed values.
  // What was in y now is in z.
  PRINT_VARn(M_Bo_W);
}

GTEST_TEST(SpatialInertia, ShiftOrigin) {
  // Place B rotated +90 degrees about W's x-axis.
  Matrix3<double> R_WB =
      AngleAxisd(M_PI_2, Vector3d::UnitX()).toRotationMatrix();

  // Spatial inertia for a thin rod of unit mass computed about its center of
  // mass and expressed in the world frame W.
  const double mass = 1.2;
  const double radius = 0.05, length = 1.0;
  SpatialInertia<double> M_Bo_W(
      mass, Vector3d::Zero(),
      mass * RotationalInertia<double>::SolidRod(radius, length));
  M_Bo_W.ReExpressInPlace(R_WB);

  // Replace this print by the respective getters and check values.
  PRINT_VARn(M_Bo_W);

  // Vector from Bo, in this case Bo = Bc, to the top of the box.
  Vector3d p_BoXo_W(0, 0.5, 0);

  PRINT_VARn(p_BoXo_W.transpose());

  // Computes spatial inertia about Xo, still expressed in W.
  SpatialInertia<double> M_Xo_W = M_Bo_W.ShiftOrigin(p_BoXo_W);

  // Replace this print by the respective checks of the new computed values.
  // What was in y now is in z.
  PRINT_VARn(M_Xo_W);

  // Checks for physically correct spatial inertia.
  EXPECT_TRUE(M_Xo_W.IsPhysicallyValid());

  // Expected moment of inertia for a rod when computed about one of its ends.
  const double I_end = mass * length / 3.0;
  const auto& I_Xo_W = M_Xo_W.get_rotational_inertia();
  EXPECT_NEAR(I_Xo_W(0,0), I_end, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(I_Xo_W(2,2), I_end, Eigen::NumTraits<double>::epsilon());
}

}
}  // math
}  // multibody
}  // drake