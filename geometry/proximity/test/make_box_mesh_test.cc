#include "drake/geometry/proximity/make_box_mesh.h"

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

// TODO(DamrongGuoy): Move the following two functions to the appropriate
//  place, so they can be shared by both make_unit_sphere_mesh_test.cc and
//  make_box_mesh_test.cc.  Right now they are duplicated.

// Computes the volume of a tetrahedron given the four vertices that define it.
// The convention is that the first three vertices a, b, c define a triangle
// with its right-handed normal pointing towards the inside of the tetrahedra.
// The fourth vertex, d, is on the positive side of the plane defined by a, b,
// c. With this convention, the computed volume will be positive, otherwise
// negative.
double CalcTetrahedronVolume(const Vector3<double>& a, const Vector3<double>& b,
                             const Vector3<double>& c,
                             const Vector3<double>& d) {
  return (d - a).dot((b - a).cross(c - a)) / 6.0;
}

// Computes the total volume of a VolumeMesh by summing up the contribution
// of each tetrahedron.
double CalcTetrahedronMeshVolume(const VolumeMesh<double>& mesh) {
  const std::vector<VolumeVertex<double>>& vertices = mesh.vertices();
  const std::vector<VolumeElement>& tetrahedra = mesh.tetrahedra();
  double volume = 0.0;
  for (const auto& t : tetrahedra) {
    double tetrahedron_volume = CalcTetrahedronVolume(
        vertices[t.vertex(0)].r_MV(), vertices[t.vertex(1)].r_MV(),
        vertices[t.vertex(2)].r_MV(), vertices[t.vertex(3)].r_MV());
    EXPECT_GT(tetrahedron_volume, 0.0);
    volume += tetrahedron_volume;
  }
  return volume;
}

GTEST_TEST(MakeBoxMeshTest, Simple) {
  Box box(0.2, 0.4, 0.8);
  VolumeMesh<double> box_mesh = MakeBoxVolumeMesh<double>::generate(box, 0.1);

  const int rectangular_cells = 2 * 4 * 8;
  const int elements_per_cell = 6;

  const int expect_num_elements = rectangular_cells * elements_per_cell;
  EXPECT_EQ(expect_num_elements, box_mesh.num_elements());

  int expect_num_vertices = 3 * 5 * 9;
  EXPECT_EQ(expect_num_vertices, box_mesh.num_vertices());

  double expect_volume = box.width() * box.depth() * box.height();
  EXPECT_NEAR(expect_volume, CalcTetrahedronMeshVolume(box_mesh),
              2.0 * std::numeric_limits<double>::epsilon());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
