#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/multibody/shapes/geometry.h"
#include "drake/soft_robots/dev/FEM/mesh.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {

std::unique_ptr<Mesh<double>> LoadTriangleMesh(const std::string& file_name) {
  DrakeShapes::Mesh mesh_loader("" /* uri for BotVisualizer */, file_name);
  // Gathers vertices and triangles from the mesh's file.
  DrakeShapes::PointsVector vertices;
  DrakeShapes::TrianglesVector triangles;
  mesh_loader.LoadObjFile(&vertices, &triangles,
                       DrakeShapes::Mesh::TriangulatePolicy::kTry);
  MatrixX<double> nodes;
  MatrixX<int> connectivities;
  const int num_elements = triangles.size();
  const int num_nodes = vertices.size();
  nodes.resize(3, num_nodes);
  connectivities.resize(3, num_elements);

  for (int inode = 0; inode < num_nodes; ++inode) {
    nodes.col(inode) = vertices[inode];
  }

  for (int iel = 0; iel < num_elements; ++iel) {
    connectivities.col(iel) = triangles[iel];
  }

  return std::make_unique<Mesh<double>>(nodes, connectivities);
}

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake