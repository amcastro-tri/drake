#pragma once

#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace drake {
namespace vtkio {

void write_vtk_mesh(const std::string& file_name,
                    const geometry::VolumeMesh<double>& mesh,
                    const std::string& title = "mesh");

void write_vtk_mesh(const std::string& file_name,
                    const geometry::SurfaceMesh<double>& mesh,
                    const std::string& title = "mesh");

}  // namespace vtkio
}  // namespace drake