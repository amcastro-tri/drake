#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"

#include <vector>
#include <fstream>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

namespace drake {
namespace geometry {
namespace internal {
namespace {

void vtk_write_header(std::ofstream& out, const std::string& title) {
    out << "# vtk DataFile Version 3.0\n";
    out << title << std::endl;
    out << "ASCII\n";
    out << std::endl;
}
void vtk_write_unstructured_grid(std::ofstream& out,
                                 const std::vector<VolumeVertex<double>>& v,
                                 const std::vector<VolumeElement>& t) {
    char message[512];
    const int numPoints = v.size();
    out << "DATASET UNSTRUCTURED_GRID\n";
    out << "POINTS " << numPoints << " double\n";
    for (int i = 0; i < numPoints; ++i) {
        const Vector3<double>& vertex = v[i].r_MV();
        sprintf(message, "%+12.8f %+12.8f %+12.8f", vertex[0], vertex[1],
                vertex[2]);
        out << message << std::endl;
    }
    const int num_tets = t.size();
    out << "CELLS " << num_tets << " " << num_tets * 5 << std::endl;
    for (int i = 0; i < num_tets; ++i) {
        const auto& tet = t[i];
        out << "4 " << tet.vertex(0) << ' ' << tet.vertex(1) << ' ' << tet.vertex(2)
            << ' ' << tet.vertex(3) << std::endl;
    }
    out << "CELL_TYPES " << num_tets << std::endl;
    for (int i = 0; i < num_tets; ++i) {
        out << "10\n";
    }
}

void write_vtk_mesh(const std::string& file_name,
                    const VolumeMesh<double>& mesh,
                    const std::string& title = "mesh") {
    std::ofstream file(file_name);
    vtk_write_header(file, title);
    vtk_write_unstructured_grid(file, mesh.vertices(), mesh.tetrahedra());
    file.close();
}

// TODO(amcastro-tri): The unit tests below were designed with the idea in mind
// that if a typical bug is present (for instance wrong tetrahedron sign
// convention, missing tetrahedra or faulty vertices computation) they would
// fail. However, from a purist point of view on unit testing, a variety of
// tests are missing. To mention a few:
//   1. Level 0 tessellation should have all of its properties confirmed:
//      tetrahedra sign convention, positive volume, sphere coverage.
//   2. Is level 0 correct in that there are no overlappong tetrahedra?
//   3. The refinement process preserves the tetrahedron sign convention.
//   4. Confirm that the k-level refinement of a tet spans the same volume of
//      the k-1-level (if there are no boundary vertices).
//
// N.B. All of these were confirmed during the development however they did not
// make it into a clean unit test.

// Computes the volume of a tetrahedron given the four vertices that define it.
// The convention is that the first three vertices a, b, c define a triangle
// with its right-handed normal pointing towards the inside of the tetrahedra.
// The fourth vertex, d, is on the positive side of the plane defined by a, b,
// c. With this convention, the computed volume will be positive, otherwise
// negative.
double CalcTetrahedronVolume(const Vector3<double>& a, const Vector3<double>& b,
                             const Vector3<double>& c,
                             const Vector3<double>& d) {
  const double tet_vol = (d - a).dot((b - a).cross(c - a)) / 6.0;
  DRAKE_DEMAND(tet_vol > 0.0);
  return tet_vol;
}

// Computes the total volume of a VolumeMesh by summing up the contribution
// of each tetrahedron.
double CalcTetrahedronMeshVolume(const VolumeMesh<double>& mesh) {
  const std::vector<VolumeVertex<double>>& vertices = mesh.vertices();
  const std::vector<VolumeElement>& tetrahedra = mesh.tetrahedra();
  double volume = 0.0;
  for (const auto& t : tetrahedra) {
    volume += CalcTetrahedronVolume(
        vertices[t.vertex(0)].r_MV(), vertices[t.vertex(1)].r_MV(),
        vertices[t.vertex(2)].r_MV(), vertices[t.vertex(3)].r_MV());
  }
  return volume;
}

GTEST_TEST(MakeCylinderMesh, RefinementTest) {

    const double height = 4.0;
    const double radius = 1.0;

  const VolumeMesh<double> mesh0 = MakeCylinderMesh<double>(height, radius,  0);
  const VolumeMesh<double> mesh1 = MakeCylinderMesh<double>(height, radius,  1);
  const VolumeMesh<double> mesh2 = MakeCylinderMesh<double>(height, radius,  2);
  const VolumeMesh<double> mesh3 = MakeCylinderMesh<double>(height, radius,  3);
  const VolumeMesh<double> mesh4 = MakeCylinderMesh<double>(height, radius,  4);

  std::cout << "writing mesh" << std::endl;

  write_vtk_mesh("cylinder0.vtk", mesh0);
  write_vtk_mesh("cylinder1.vtk", mesh1);
  write_vtk_mesh("cylinder2.vtk", mesh2);
  write_vtk_mesh("cylinder3.vtk", mesh3);
  write_vtk_mesh("cylinder4.vtk", mesh4);

  const double expected_volume = M_PI * radius * radius * height;

  PRINT_VAR(expected_volume);

  PRINT_VAR(CalcTetrahedronMeshVolume(mesh0));
  PRINT_VAR(CalcTetrahedronMeshVolume(mesh1));
  PRINT_VAR(CalcTetrahedronMeshVolume(mesh2));
  PRINT_VAR(CalcTetrahedronMeshVolume(mesh3));
  PRINT_VAR(CalcTetrahedronMeshVolume(mesh4));

  PRINT_VAR(mesh0.num_vertices());
  PRINT_VAR(mesh0.num_elements());
  PRINT_VAR(mesh1.num_vertices());
  PRINT_VAR(mesh1.num_elements());
  PRINT_VAR(mesh2.num_vertices());
  PRINT_VAR(mesh2.num_elements());
  PRINT_VAR(mesh3.num_vertices());
  PRINT_VAR(mesh3.num_elements());
  PRINT_VAR(mesh4.num_vertices());
  PRINT_VAR(mesh4.num_elements());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
