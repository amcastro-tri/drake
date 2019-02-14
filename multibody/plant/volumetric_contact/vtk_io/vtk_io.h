#pragma once

#include <memory>
#include <vector>
#include <fstream>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"

namespace drake {
namespace multibody {

void vtk_write_header(std::ofstream& out, const std::string& title) {
    out << "# vtk DataFile Version 3.0\n";
    out << title << std::endl;
    out << "ASCII\n";
    out << std::endl;
}

void vtk_write_unstructured_grid(std::ofstream& out, const Wm5::ConvexPolyhedron<double>& poly) {
    char message[512];

    const auto& points = poly.GetPoints();
    const int numPoints = points.size();
    out << "DATASET UNSTRUCTURED_GRID\n";
    out << "POINTS " << numPoints << " double\n";
    for (int i = 0; i < numPoints; ++i)
    {
        const Wm5::Vector3<double>& vertex = points[i];
        sprintf(message, "%+12.8f %+12.8f %+12.8f",
            vertex.X(), vertex.Y(), vertex.Z());
        out << message << std::endl;
    }
    
    const int numTriangles = poly.GetNumTriangles();
    out << "CELLS " << numTriangles << " " << numTriangles*4 << std::endl;
    for (int t = 0; t < numTriangles; ++t)
    {
        const MTTriangle& triangle = poly.GetTriangle(t);

        // Zero based.
        const int p0 = poly.GetVertex(triangle.GetVertex(0)).GetLabel();
        const int p1 = poly.GetVertex(triangle.GetVertex(1)).GetLabel();
        const int p2 = poly.GetVertex(triangle.GetVertex(2)).GetLabel();

        out << "3 " << p0 << ' ' << p1 << ' '<< p2 << std::endl;
    }


    
    out << "CELL_TYPES " << numTriangles << std::endl;
    for (int t = 0; t < numTriangles; ++t) {
        out << "5\n";
    }
}

void vtk_write_cell_data_header(std::ofstream& out, int ntris) {
   out << "CELL_DATA " << ntris << std::endl;
}

void vtk_write_scalar_data(std::ofstream& out, const std::string& scalar_name, const std::vector<int>& scalars) {
   out << "SCALARS " << scalar_name << " double\n";
   out << "LOOKUP_TABLE default\n";
   for (const auto s : scalars) out << s << std::endl;
}


}  // namespace multibody
}  // namespace drake
