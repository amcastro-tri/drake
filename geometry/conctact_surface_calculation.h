#pragma once

//#include <limits>
#include <memory>
//#include <unordered_set>
//#include <vector>

//#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/surface_mesh.h"
#include "drake/geometry/tetrahedral_volume_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
class ContactSurfaceCalculator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurfaceCalculator)

  static SurfaceMesh<T> CalcContactSurfaceBetweenMeshAndLevelSet(
      const TetrahedralVolumeMesh<T>& mesh_M,
      std::function<T(const Vector3<T>&)> phi_N,
      const math::RigidTransform<T>& X_NM) {
    const int num_vertices = mesh_M.vertices.size();
    VectorX<T> phiN_at_V(num_vertices);
    for (int i = 0; i < num_vertices; ++i) {
      const Vector3<T>& p_MV = mesh_M.vertices[i];
      const Vector3<T> p_NV = X_NM * p_MV;
      phiN_at_V[i] = phi_N(p_NV);
    }

    std::vector<SurfaceFace> faces;
    std::vector<SurfaceVertex<T>> vertices

    const int num_tets = mesh_M.tetrahedra.size();
    for (int t = 0; t < num_tets; ++t) {
      const Vector4<int>& tet = mesh_M.tetrahedra[t];
      // Level set for N evaluated at each tet corner.
      Vector4<T> tet_phi;
      for (int i = 0; i < 4; ++i) tet_phi[i] = phiN_at_V[tet[i]];

      const bool tet_is_outside_N = (tet_phi.array() > 0.0).all();
      const bool tet_is_inside_N = (tet_phi.array() < 0.0).all();
      const bool tet_is_intersected = !tet_is_inside_N && !tet_is_outside_N;

      if (tet_is_intersected) {
        // Scan each edge and find out if the level set crosses zero.
        // There are at most 6 zero crossing points, one per edge.
        std::array<Vector3<T>, 6> zero_crossings;
        int num_zero_crossings = 0;
        // the geometric center of the zero-crossing points.
        Vector3<T> p_MC = Vector3<T>::Zero();
        for (const auto& e : edges_) {
          // phi_N evaluated at the two vertices of the edge.
          const T phi_v1 = tet_phi[e.first];
          const T phi_v2 = tet_phi[e.second];

          // There is intersection. Compute the zero-crossing on phi_N.
          if (phi_v1 * phi_v2 < 0.0) {
            using std::abs;
            // Compute weights for linear interpolation.
            const T phi_v1_abs = abs(phi_v1);
            const T phi_v2_abs = abs(phi_v2);
            const T phi_12 = phi_v1_abs + phi_v2_abs;
            const T w1 = phi_v1_abs / phi_12;
            const T w2 = 1.0 - w1;

            // Get edge vertices.
            const int v1 = tet[e.first];
            const int v2 = tet[e.second];
            const auto& p_MV1 = mesh_M.vertices[v1];
            const auto& p_MV2 = mesh_M.vertices[v1];

            // Interpolate the zero-crossing point Z between V1 and V2.
            const Vector3<T> p_MZ = w1 * p_MV1 + w2 * p_MV2;

            zero_crossings[num_zero_crossings++] = p_MZ;
            p_MC += p_MZ;
          }
        }
        p_MC /= num_zero_crossings;

        // Create triangles from the zero-crossing points.
        // We connect each zero-crossing to their geometric center to make a fan
        // of triangles.
        for (int i = 0; i < num_zero_crossings; ++i) {
            vertices.push_back(zero_crossings[i]);
            
        }


      }
    }
  }

 private:
  // The six edges of a tetrahedra
  const std::array<std::pair<int, int>, 6> edges_{
      {0, 1}, {1, 2}, {2, 0},   // base formed by vertices 0, 1, 2.
      {0, 3}, {1, 3}, {2, 3}};  // pyramid with top at node 3.
};

}  // namespace internal.
}  // namespace geometry.
}  // namespace drake.
