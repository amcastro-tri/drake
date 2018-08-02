#include <memory>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/mesh_query/vtk_io.h"
#include "drake/geometry/mesh_query/mesh_query.h"
#include "drake/multibody/shapes/geometry.h"

namespace drake {
namespace geometry {
namespace mesh_query {
namespace {

using Eigen::AngleAxisd;
using Eigen::Translation3d;
using Eigen::Isometry3d;
using Eigen::Vector3d;

std::unique_ptr<Mesh<double>> LoadMeshFromObj(
    const std::string& file_name) {
  const auto resource_name = FindResourceOrThrow(file_name);
  DrakeShapes::Mesh mesh_loader(resource_name, resource_name);

  auto mesh = std::make_unique<Mesh<double>>();
  mesh_loader.LoadObjFile(&mesh->points_G, &mesh->triangles);

  // Compute normals.
  mesh->face_normals_G = CalcMeshFaceNormals(mesh->points_G, mesh->triangles);
  mesh->node_normals_G = CalcAreaWeightedNormals(*mesh);

  const int num_points = mesh->points_G.size();
  // Allocate and initialize to invalid index values.
  mesh->node_element.resize(num_points, std::make_pair(-1, -1));

  // Arbitrarily fill in mesh->node_element.
  const int num_elements = mesh->triangles.size();
  for (int element_index = 0; element_index < num_elements; ++element_index) {
    const auto& triangle = mesh->triangles[element_index];
    for (int i = 0; i < 3; ++i) {
      const int node_index = triangle[i];
      if (mesh->node_element[node_index].first < 0) {  // not yet initialized.
        mesh->node_element[node_index].first = element_index;
        mesh->node_element[node_index].second = i;
      }
    }
  }

  return mesh;
}

int DoMain() {

  std::unique_ptr<Mesh<double>> sphere = LoadMeshFromObj(
      "drake/geometry/mesh_query/examples/sphere.obj");
  sphere->mesh_index = 0;

  std::unique_ptr<Mesh<double>> ellipsoid = LoadMeshFromObj(
      "drake/geometry/mesh_query/examples/ellipsoid.obj");
  ellipsoid->mesh_index = 1;

  // For this example normals in the obj point inward and therefore we flip
  // them.
  FlipNormals(sphere.get());
  FlipNormals(ellipsoid.get());

  const double radius = 1.0;
  const double penetration = 0.02;
  const double z_WSo = radius - penetration;

  // Place sphere a "penetration" distance below z = 0.
  // Apply an arbirary rotation for testing.
  Isometry3d X_WSphere{Translation3d{Vector3d(0, 0, z_WSo)}};
  X_WSphere.linear() =
      (AngleAxisd(M_PI / 3., Vector3d::UnitX()) *
       AngleAxisd(M_PI / 5., Vector3d::UnitY()) *
       AngleAxisd(3. * M_PI / 8., Vector3d::UnitZ())).matrix();

  // The top of the ellipsoid is placed at z = 0
  Isometry3d X_WEllipsoid{Translation3d{Vector3d(0, 0, -1.0)}};
  X_WEllipsoid.linear() = AngleAxisd(M_PI_2, Vector3d::UnitX()).matrix();

  // Write mesh and normals to a file.
  std::ofstream bottom_sphere_file("ellipsoid.vtk");
  OutputMeshToVTK(bottom_sphere_file, ellipsoid->points_G, ellipsoid->triangles, X_WEllipsoid);
  AppendCellCenteredVectorFieldToVTK(
      bottom_sphere_file, "FaceNormals", ellipsoid->face_normals_G, X_WEllipsoid);
  AppendNodeCenteredVectorFieldToVTK(
      bottom_sphere_file, "NodeNormals", ellipsoid->node_normals_G, X_WEllipsoid);
  bottom_sphere_file.close();

  std::ofstream top_sphere_file("sphere.vtk");
  OutputMeshToVTK(top_sphere_file, sphere->points_G, sphere->triangles, X_WSphere);
  AppendCellCenteredVectorFieldToVTK(
      top_sphere_file, "FaceNormals", sphere->face_normals_G, X_WSphere);
  AppendNodeCenteredVectorFieldToVTK(
      top_sphere_file, "NodeNormals", sphere->node_normals_G, X_WSphere);
  top_sphere_file.close();

  // Perform the mesh-mesh query.
  std::vector<PenetrationAsTrianglePair<double>> results = MeshToMeshQuery(
      X_WEllipsoid, *ellipsoid,
      X_WSphere, *sphere);

  std::vector<Vector3d> pointsA(results.size());
  std::transform(results.begin(), results.end(), pointsA.begin(),
                 [](const PenetrationAsTrianglePair<double>& results) {
                   return results.p_WoAs_W;
                 });

  std::vector<Vector3d> pointsB(results.size());
  std::transform(results.begin(), results.end(), pointsB.begin(),
  [](const PenetrationAsTrianglePair<double>& results) {
    return results.p_WoBs_W;
  });

  std::vector<Vector3d> normals;
  for (const auto& result : results) {
    normals.push_back(result.normal_A_W);
    normals.push_back(result.normal_B_W);
  }

  {
    std::ofstream file("pairs.vtk");
    OutputSegmentsToVTK(file, pointsA, pointsB);
    AppendNodeCenteredVectorFieldToVTK(
        file, "normals", normals);
    file.close();
  }

  return 0;
}

}  // namespace
}  // namespace mesh_query
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::mesh_query::DoMain();
}
