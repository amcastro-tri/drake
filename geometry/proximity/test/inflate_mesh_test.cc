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
#include "drake/geometry/proximity/make_mesh_field.h"

#include <iostream>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;

std::vector<double> CalcCellVolumes(const VolumeMesh<double>& mesh) {
  std::vector<double> tet_volumes(mesh.num_elements());
  for (int e = 0; e < mesh.num_elements(); ++e) {
    tet_volumes[e] = mesh.CalcTetrahedronVolume(e);
  }
  return tet_volumes;
}

VolumeMesh<double> RemoveNegativeVolumes(const VolumeMesh<double>& mesh) {  
  std::vector<VolumeElement> tets;
  for (int e = 0; e < mesh.num_elements(); ++e) {
    const double vol = mesh.CalcTetrahedronVolume(e);
    if (vol > 0) {
      tets.push_back(mesh.element(e));
    }
  }
  std::vector<Vector3d> verts = mesh.vertices();

  std::cout << fmt::format("All tets: {}\n", mesh.num_elements());
  std::cout << fmt::format("Positive tets: {}\n", tets.size());

  return VolumeMesh<double>(std::move(tets), std::move(verts));
}

GTEST_TEST(MakeInflatedMesh, NonConvex) {
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;

  const double kHydroelasticModulus = 1.0;  
  const double kMargin = 1e-3;
  //const VolumeMesh<double> mesh = MakeVolumeMeshFromVtk<double>(
  //    Mesh(FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk")));

  //const std::string model = "drake/geometry/test/non_convex_mesh.vtk";
  //const std::string model = "drake/geometry/test/sweet_home_dish_drying_rack_wireframe_low.vtk";
  const std::string model = "drake/geometry/test/vivo_custom_single_shelf_low.vtk";
  //const std::string model = "drake/geometry/test/green_anjou_pear_low.vtk";

  const VolumeMesh<double> mesh =
      MakeVolumeMeshFromVtk<double>(Mesh(FindResourceOrThrow(model)));

  std::vector<int> new_vertices;
  auto start = high_resolution_clock::now();
  const VolumeMesh<double> inflated_mesh =
      MakeInflatedMesh(mesh, kMargin, &new_vertices);
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

  const VolumeMesh<double> positive_mesh = RemoveNegativeVolumes(inflated_mesh);
  WriteVolumeMeshToVtk("positive_mesh.vtk", positive_mesh, "positive_mesh");

  (void)kHydroelasticModulus;

#if 0
  WriteCellCenteredScalarFieldToVtk("mesh_volumes.vtk", "cell volume", mesh,
                                    volumes, "Inflated mesh");
  WriteCellCenteredScalarFieldToVtk("inflated_mesh_volumes.vtk", "cell volume",
                                    mesh, inflated_volumes, "Inflated mesh");
#endif                                    

  // Make field on the original mesh.
  const VolumeMeshFieldLinear<double, double> field =
      MakeVolumeMeshPressureField(&mesh, kHydroelasticModulus, kMargin);

  // Make field on the inflated mesh.
  std::vector<double> values = field.values();
  for (int v : new_vertices) {
    values.push_back(values[v]);
  }
  ASSERT_EQ(values.size(), inflated_mesh.num_vertices());
  std::vector<double> values_copy = values;

  VolumeMeshFieldLinear<double, double> inflated_field(
      std::move(values), &inflated_mesh,
      MeshGradientMode::
          kOkOrThrow /* what MakeVolumeMeshPressureField() uses. */);

  WriteVolumeMeshFieldLinearToVtk("pressure_on_original_mesh.vtk", "pressure",
                                  field, "Pressure on original mesh");
  WriteVolumeMeshFieldLinearToVtk("pressure_on_inflated_mesh.vtk", "pressure",
                                  inflated_field, "Pressure on inflated mesh");

  VolumeMeshFieldLinear<double, double> positive_field(
      std::move(values_copy), &positive_mesh,
      MeshGradientMode::
          kOkOrThrow /* what MakeVolumeMeshPressureField() uses. */);
  WriteVolumeMeshFieldLinearToVtk("pressure_on_positive_mesh.vtk", "pressure",
                                  positive_field, "Pressure on positive mesh");
}
}
}
}
}
