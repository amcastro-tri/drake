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

using Eigen::Vector3d;

GTEST_TEST(MassProperties, RotationalInertia) {
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

  std::cout << I << std::endl;

  PRINT_VARn(I.get_matrix());
  //std::cout << I.get_matrix();
}

}
}  // math
}  // multibody
}  // drake