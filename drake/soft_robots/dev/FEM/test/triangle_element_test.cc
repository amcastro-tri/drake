#include "drake/soft_robots/dev/FEM/triangle_element.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {
namespace {

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// Loads a mesh and verifies sizes.
GTEST_TEST(TriangleElement, CalcShapeFunctionsAtCorners) {
  TriangleElement<double, 3> element;
  Matrix<double, 2, 3> xcorners;
  xcorners <<
           0.0, 1.0, 0.0, /* r */
           0.0, 0.0, 1.0; /* s */
  MatrixX<double> Na(element.get_num_nodes(), xcorners.cols());
  element.CalcShapeFunctions(xcorners, Na);
  MatrixX<double> Na_expected(element.get_num_nodes(), xcorners.cols());
  Na_expected <<
              Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ();
  EXPECT_TRUE(Na.isApprox(Na_expected));
}

GTEST_TEST(TriangleElement, CalcShapeFunctionsAtHalfEdges) {
  TriangleElement<double, 3> element;
  Matrix<double, 2, 3> xcorners;
  xcorners <<
           0.5, 0.0, 0.5, /* r */
           0.0, 0.5, 0.5; /* s */
  MatrixX<double> Na(element.get_num_nodes(), xcorners.cols());
  element.CalcShapeFunctions(xcorners, Na);
  MatrixX<double> Na_expected(element.get_num_nodes(), xcorners.cols());
  Na_expected <<
              0.5, 0.5, 0.0,
              0.5, 0.0, 0.5,
              0.0, 0.5, 0.5;
  EXPECT_TRUE(Na.isApprox(Na_expected));
}

GTEST_TEST(TriangleElement, CalcJacobianNorm) {
  TriangleElement<double, 3> element;

  // Element nodes.
  Matrix<double, 3, 3> xa;
  xa <<
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 0.0;

  // Some quadrature points where to evaluate.
  Matrix<double, 2, 3> xquad;
  xquad <<
           0.5, 0.0, 0.5, /* r */
           0.0, 0.5, 0.5; /* s */
  VectorX<double> J(xquad.cols());
  element.CalcJacobianNorm(xa, xquad, J);
  VectorX<double> J_expected(xquad.cols());
  J_expected << 0.5, 0.5, 0.5;
  EXPECT_TRUE(J.isApprox(J_expected));
}


}  // namespace
}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake