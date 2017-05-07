#include "drake/soft_robots/dev/FEM/mesh.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/soft_robots/dev/FEM/mesh_loader.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {
namespace {

using Eigen::Vector3d;

// Loads a mesh and verifies sizes.
GTEST_TEST(Mesh, MeshLoader) {
  const std::string file_name = drake::GetDrakePath() +
      "/multibody/shapes/test/tri_cube.obj";
  std::unique_ptr<Mesh<double>> mesh = LoadTriangleMesh(file_name);
  EXPECT_EQ(mesh->get_num_nodes(), 8);
  EXPECT_EQ(mesh->get_num_elements(), 12);
  EXPECT_EQ(mesh->get_num_physical_dimensions(), 3);
  Vector3<int> element0 = mesh->get_element_connectivities(0);
  EXPECT_EQ(element0, Vector3<int>(0, 2, 3));
  Vector3<int> element3 = mesh->get_element_connectivities(3);
  EXPECT_EQ(element3, Vector3<int>(5, 2, 1));
  Matrix3<double> xelement3;
  mesh->GetElementNodes(3, xelement3);
  EXPECT_TRUE(xelement3.col(0).isApprox(Vector3d(1.0, 1.0, 1.0)));
  EXPECT_TRUE(xelement3.col(1).isApprox(Vector3d(-1.0, -1.0, 1.0)));
  EXPECT_TRUE(xelement3.col(2).isApprox(Vector3d(1.0, -1.0, 1.0)));
}

}  // namespace
}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake