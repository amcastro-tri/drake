#include "drake/soft_robots/dev/FEM/triangle_element.h"

#include <limits>
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
GTEST_TEST(TriangleElement3D, CalcShapeFunctionsAtCorners) {
  TriangleElement3D<double> element;
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

GTEST_TEST(TriangleElement3D, CalcShapeFunctionsAtHalfEdges) {
  TriangleElement3D<double> element;
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

GTEST_TEST(TriangleElement3D, CalcJacobianNorm) {
  TriangleElement3D<double> element;

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

GTEST_TEST(TriangleElement3D, AreaVector) {
  TriangleElement3D<double> element;

  // Element nodes.
  Matrix<double, 3, 3> xa;
  // Triangle defined by points:
  // x1 = [1, 0, 0]^T;
  // x2 = [0, 1, 0]^T;
  // x3 = [0, 0, 1]^T;
  xa <<
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0;

  Vector3d S = element.CalcAreaVector(xa);

  // Compute area by Herons formula.
  const double side_length = sqrt(2.0);
  const double semiperimeter = 3.0 * side_length / 2.0;
  const double c1 = semiperimeter - side_length;
  const double area_by_heron = sqrt(semiperimeter * c1 * c1 * c1);

  EXPECT_NEAR(element.CalcArea(xa), area_by_heron,
              2.0 * std::numeric_limits<double>::epsilon());

  Vector3<double> S_expected(1, 1, 1);
  S_expected.normalize();  // Now the magnitude is one.
  S_expected *= area_by_heron;

  EXPECT_TRUE(S.isApprox(S_expected));
}

}  // namespace
}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake