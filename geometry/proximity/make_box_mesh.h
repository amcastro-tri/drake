#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

template<typename T>
class MakeBoxVolumeMesh {
 public:
  /** Generates a tetrahedral volume mesh of a given box. The vertices are
   unique, so adjacent tetrahedra use their shared vertices, instead of
   repeating the same vertex coordinates in the list of vertices multiple
   times.
  */
  static VolumeMesh<T> generate(const Box& box, T target_edge_length);

 private:
  static std::vector<T> UniformSample(T first, T last, int num) {
    DRAKE_DEMAND(num >= 2);
    T sample = first;
    const T delta = (last - first) / (num - 1);
    std::vector<T> samples(num);
    // We could have assigned samples[num - 1] in this for loop by "i < num", but
    // accumulation errors in "sample += delta" may deviate `sample` from `last`.
    // Thus, we have "i < num - 1", and assign `last` to samples[num - 1]
    // explicitly after the for loop.
    for (int i = 0; i < num - 1; ++i) {
      samples[i] = sample;
      sample += delta;
    }
    samples[num - 1] = last;
    return samples;
  }

  // TODO(DamrongGuoy): Switch to something like boost::multi_array.
  //  We want "int vertex_index[nx][ny][nz]", but it can't be passed to a
  //  subroutine and still use operators [][][] conveniently. This is a
  //  less-efficient work around.
  typedef std::vector<int> Array1;
  typedef std::vector<std::vector<int>> Array2;
  typedef std::vector<std::vector<std::vector<int>>> Array3;

  static std::vector<VolumeVertex<T>> GenerateVertices(const Box& box,
                                                       const int nx,
                                                       const int ny,
                                                       const int nz,
                                                       Array3* vertex_index) {
    T half_x = box.width() / T(2);
    T half_y = box.depth() / T(2);
    T half_z = box.height() / T(2);
    auto x_coords = UniformSample(-half_x, half_x, nx);
    auto y_coords = UniformSample(-half_y, half_y, ny);
    auto z_coords = UniformSample(-half_z, half_z, nz);

    std::vector<VolumeVertex<T>> vertices;
    for (int i = 0; i < nx; ++i) {
      for (int j = 0; j < ny; ++j) {
        for (int k = 0; k < nz; ++k) {
          (*vertex_index)[i][j][k] = vertices.size();
          vertices.emplace_back(x_coords[i], y_coords[j], z_coords[k]);
        }
      }
    }
    return vertices;
  }

  static std::vector<VolumeElement> GenerateElements(
      const int nx, const int ny, const int nz, const Array3& vertex_index) {
    // We use this table to subdivide a topological cube into six congruent
    // tetrahedra. The table represents a ring of six vertices on a topological
    // cube. Two consecutive vertices in the ring together with the main
    // diagonal of the cube ({0,0,0},{1,1,1}) forms a tetrahedron. The seventh
    // entry in the table is the repeat of the first entry for convenience.
    // TODO(DamrongGuoy): Draw a picture to explain this table.
    int vv[7][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1},
                    {0, 0, 1}, {1, 0, 1}, {1, 0, 0}};
    std::vector<VolumeElement> elements;
    for (int i = 0; i < nx - 1; ++i) {
      for (int j = 0; j < ny - 1; ++j) {
        for (int k = 0; k < nz - 1; ++k) {
          const int a = vertex_index[i][j][k];
          const int b = vertex_index[i + 1][j + 1][k + 1];
          for (int l = 0; l < 6; ++l) {
            const int s[3] = {vv[l][0], vv[l][1], vv[l][2]};
            const int t[3] = {vv[l + 1][0], vv[l + 1][1], vv[l + 1][2]};
            const int c = vertex_index[i + s[0]][j + s[1]][k + s[2]];
            const int d = vertex_index[i + t[0]][j + t[1]][k + t[2]];
            const int vertices[4] = {a, b, c, d};
            elements.emplace_back(vertices);
          }
        }
      }
    }

    return elements;
  }

};  // class MakeBoxVolumeMesh

template <typename T>
VolumeMesh<T> MakeBoxVolumeMesh<T>::generate(const Box& box,
                                             T target_edge_length) {
  DRAKE_DEMAND(target_edge_length > T(0));
  int nx = 1 + static_cast<int>(ceil(box.width() / target_edge_length));
  int ny = 1 + static_cast<int>(ceil(box.depth() / target_edge_length));
  int nz = 1 + static_cast<int>(ceil(box.height() / target_edge_length));

  // TODO(DamrongGuoy): Switch to something like boost::multi_array.
  //  We want "int vertex_index[nx][ny][nz]", but it can't be passed to a
  //  subroutine and still use operators [][][] conveniently. This is a
  //  less-efficient work around.
  Array3 vertex_index(nx, Array2(ny, Array1(nz)));

  std::vector<VolumeVertex<T>> vertices =
      GenerateVertices(box, nx, ny, nz, &vertex_index);

  std::vector<VolumeElement> elements =
      GenerateElements(nx, ny, nz, vertex_index);

  return VolumeMesh<T>(std::move(elements), std::move(vertices));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
