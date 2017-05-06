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

// Loads a mesh and verifies sizes.
GTEST_TEST(Mesh, MeshLoader) {
  const std::string file_name = drake::GetDrakePath() +
      "/multibody/shapes/test/tri_cube.obj";
  std::unique_ptr<Mesh<double>> mesh = LoadTriangleMesh(file_name);
  EXPECT_EQ(mesh->get_num_nodes(), 8);
  EXPECT_EQ(mesh->get_num_elements(), 12);
  EXPECT_EQ(mesh->get_num_physical_dimensions(), 3);
}

}  // namespace
}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake