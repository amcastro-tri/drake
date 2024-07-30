#include "drake/geometry/proximity/inflate_mesh.h"

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <fstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/make_mesh_from_vtk.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"
#include "drake/geometry/shape_specification.h"

#include <iostream>

namespace drake {
namespace geometry {
namespace internal {
namespace {

//using Eigen::Vector3d;

std::vector<double> CalcCellVolumes(const VolumeMesh<double>& mesh) {
  std::vector<double> tet_volumes(mesh.num_elements());
  for (int e = 0; e < mesh.num_elements(); ++e) {
    tet_volumes[e] = mesh.CalcTetrahedronVolume(e);
  }
  return tet_volumes;
}

GTEST_TEST(MakeConvexHullMeshTest, CubeWithHole) {
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;

  const double kMargin = 2e-3;
  //const VolumeMesh<double> mesh = MakeVolumeMeshFromVtk<double>(
  //    Mesh(FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk")));

  //const std::string model = "drake/geometry/test/non_convex_mesh.vtk";
  const std::string model = "drake/geometry/test/sweet_home_dish_drying_rack_wireframe_low.vtk";
  //const std::string model = "drake/geometry/test/green_anjou_pear_low.vtk";

  const VolumeMesh<double> mesh =
      MakeVolumeMeshFromVtk<double>(Mesh(FindResourceOrThrow(model)));

  auto start = high_resolution_clock::now();      
  const VolumeMesh<double> inflated_mesh = MakeInflatedMesh(mesh, kMargin);
  auto duration =
      duration_cast<microseconds>(high_resolution_clock::now() - start);
  std::cout << fmt::format("MakeInflatedMesh: {} s\n", duration.count() / 1e6);

  WriteVolumeMeshToVtk("mesh.vtk", mesh, "mesh");  
  WriteVolumeMeshToVtk("inflated_mesh.vtk", inflated_mesh, "inflated_mesh");

  std::vector<double> volumes = CalcCellVolumes(mesh);

  double min_vol = std::numeric_limits<double>::max();
  for (int e = 0; e < mesh.num_elements(); ++e) {
    const double V = volumes[e];
    min_vol = std::min(min_vol, V);
  }
  std::cout << fmt::format("non-inflated: {}\n", min_vol);

  std::vector<double> inflated_volumes = CalcCellVolumes(inflated_mesh);

  min_vol = std::numeric_limits<double>::max();
  for (int e = 0; e < mesh.num_elements(); ++e) {
    const double V = inflated_volumes[e];
    min_vol = std::min(min_vol, V);
  }
  std::cout << fmt::format("inflated: {}\n", min_vol);

  WriteCellCenteredScalarFieldToVtk("mesh_volumes.vtk", "cell volume", mesh,
                                    volumes, "Inflated mesh");
  WriteCellCenteredScalarFieldToVtk("inflated_mesh_volumes.vtk", "cell volume",
                                    mesh, inflated_volumes, "Inflated mesh");
}
}
}
}
}
