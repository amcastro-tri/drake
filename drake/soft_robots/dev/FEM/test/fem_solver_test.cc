#include "drake/soft_robots/dev/FEM/fem_solver.h"

#include <memory>

//#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/soft_robots/dev/FEM/mesh_loader.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace soft_robots {
namespace drake_fem {
namespace {

using Eigen::Vector3d;

double fun(const Eigen::Ref<const MatrixX<double>>& x) {
  return 1.0;
}

// Loads a mesh and verifies sizes.
//GTEST_TEST(Mesh, MeshLoader) {
void run() {
  const std::string file_name = drake::GetDrakePath() +
      "/multibody/shapes/test/tri_cube.obj";
  std::unique_ptr<Mesh<double>> mesh = LoadTriangleMesh(file_name);
  //EXPECT_EQ(mesh->get_num_nodes(), 8);
  //EXPECT_EQ(mesh->get_num_elements(), 12);
  //EXPECT_EQ(mesh->get_num_physical_dimensions(), 3);

  FEMSolver<double> solver(mesh.get());

  double integral;
  double area;
  solver.ComputeIntegralOfScalarField(
      fun, &integral, &area);

  PRINT_VAR(integral);
  PRINT_VAR(area);

}

}  // namespace
}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake

int main() {
  drake::soft_robots::drake_fem::run();
  return 0;
}