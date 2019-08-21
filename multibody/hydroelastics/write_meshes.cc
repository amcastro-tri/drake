#include "drake/multibody/hydroelastics/write_meshes.h"

namespace drake {
namespace vtkio {

void vtk_write_header(std::ofstream& out, const std::string& title) {
  out << "# vtk DataFile Version 3.0\n";
  out << title << std::endl;
  out << "ASCII\n";
  out << std::endl;
}

void vtk_write_unstructured_grid(
    std::ofstream& out, const std::vector<geometry::VolumeVertex<double>>& v,
    const std::vector<geometry::VolumeElement>& t) {
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

void vtk_write_unstructured_grid(std::ofstream& out,
                                 const geometry::SurfaceMesh<double>& mesh) {
  char message[512];

  const int numPoints = mesh.num_vertices();
  out << "DATASET UNSTRUCTURED_GRID\n";
  out << "POINTS " << numPoints << " double\n";
  for (geometry::SurfaceVertexIndex i(0); i < numPoints; ++i) {
    const Vector3<double>& vertex = mesh.vertex(i).r_MV();
    sprintf(message, "%+12.8f %+12.8f %+12.8f", vertex[0], vertex[1],
            vertex[2]);
    out << message << std::endl;
  }

  const int num_tets = mesh.num_faces();
  out << "CELLS " << num_tets << " " << num_tets * 4 << std::endl;
  for (geometry::SurfaceFaceIndex i(0); i < num_tets; ++i) {
    const auto& tet = mesh.element(i);
    out << "3 " << tet.vertex(0) << ' ' << tet.vertex(1) << ' ' << tet.vertex(2)
        << std::endl;
  }

  out << "CELL_TYPES " << num_tets << std::endl;
  for (int i = 0; i < num_tets; ++i) {
    out << "5\n";
  }
}

void write_vtk_mesh(const std::string& file_name,
                    const geometry::SurfaceMesh<double>& mesh,
                    const std::string& title) {
  std::ofstream file(file_name);
  vtk_write_header(file, title);
  vtk_write_unstructured_grid(file, mesh);
  file.close();
}

}  // namespace vtkio
}  // namespace drake
