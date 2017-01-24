#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/multibody/multibody_tree/mass_properties.h"

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

  RotationalInertia<double> I_Ro_F = I_Ro_R.ReExpressIn(R_FR);

  // Now the R's z-axis is oriented along F's y-axis.
  EXPECT_NEAR(I_Ro_F(0, 0), Iperp, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(I_Ro_F(1, 1), Irr, Eigen::NumTraits<double>::epsilon());
  EXPECT_NEAR(I_Ro_F(2, 2), Iperp, Eigen::NumTraits<double>::epsilon());

  PRINT_VARn(I_Ro_F);

}

}
}  // math
}  // multibody
}  // drake