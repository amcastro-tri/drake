#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"

#include <vector>
#include <fstream>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

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


GTEST_TEST(MakeCylinderMesh, RefinementTest) {

  const VolumeMesh<double>& mesh0 = MakeCylinderMesh<double>(4, 1, 0);
  const VolumeMesh<double>& mesh1 = MakeCylinderMesh<double>(4, 1, 1);
  const VolumeMesh<double>& mesh2 = MakeCylinderMesh<double>(4, 1, 2);
  const VolumeMesh<double>& mesh3 = MakeCylinderMesh<double>(4, 1, 3);
  const VolumeMesh<double>& mesh4 = MakeCylinderMesh<double>(4, 1, 4);

  std::cout << "writing mesh" << std::endl;

  write_vtk_mesh("cylinder0.vtk", mesh0);
  write_vtk_mesh("cylinder1.vtk", mesh1);
  write_vtk_mesh("cylinder2.vtk", mesh2);
  write_vtk_mesh("cylinder3.vtk", mesh3);
  write_vtk_mesh("cylinder4.vtk", mesh4);

}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
