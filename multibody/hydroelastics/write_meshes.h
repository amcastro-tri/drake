#pragma once

#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace drake {
namespace vtkio {

void vtk_write_header(std::ofstream& out, const std::string& title);
void vtk_write_unstructured_grid(
    std::ofstream& out, const std::vector<geometry::VolumeVertex<double>>& v,
    const std::vector<geometry::VolumeElement>& t);    

// SFINAE to crate a dummy implementation when T != double.
template <typename T>
typename std::enable_if<
      !std::is_same<T, double>::value>::type
write_vtk_mesh(const std::string& ,
                    const geometry::VolumeMesh<T>& ,
                    const std::string& title = "mesh") {
  throw std::runtime_error("blah: " + title);
}

template <typename T>
typename std::enable_if<
      std::is_same<T, double>::value>::type
write_vtk_mesh(const std::string& file_name,
                    const geometry::VolumeMesh<T>& mesh,
                    const std::string& title = "mesh") {
  std::ofstream file(file_name);
  vtk_write_header(file, title);
  vtk_write_unstructured_grid(file, mesh.vertices(), mesh.tetrahedra());
  file.close();
}

void write_vtk_mesh(const std::string& file_name,
                    const geometry::SurfaceMesh<double>& mesh,
                    const std::string& title = "mesh");


}  // namespace vtkio
}  // namespace drake