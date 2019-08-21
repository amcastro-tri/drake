#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/multibody/hydroelastics/hydroelastic_field.h"

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {

/// Creates a HydroelasticField for a sphere of a given radius.
/// The input parameter `refinement_level`, ℓ ∈ ℕ₀, controls the resolution of
/// the mesh generated. The resulting number of tetrahedra nₜ can
/// be predicted according to: nₜ = 8ˡ⁺¹. A characteristic tetrahedron length
/// can be estimated with `h  = R / (ℓ + 1)`, with R the sphere radius.
/// Even though the dimensionless hydroelastic strain field is arbitrary, we
/// construct it to satisfy a number of properties:
///  1. The field ε is zero at the boundary and increases towards the interior.
///  2. The field ε and its gradient ∇ε are continuous in the domain of the
///     sphere.
///  3. The radial gradient at the boundary of the sphere is
///     dε/dr(r = R) = -1 / R, with R the radius of the sphere, so that if the
///     field was linear, then the maximum strain ε = 1 would occur at the
///     center of the sphere.
///
/// We then choose the simplest functional form that can satisfy these
/// requirements; a quadratic function of the radius, ε(r) = 0.5 [1 - (r / R)²].
template <typename T>
std::unique_ptr<HydroelasticField<T>> MakeBoxHydroelasticField(
    const geometry::Box& box, double target_edge_length) {
  const Vector3<double> half_sizes = box.size() / 2.0;
  const double min_half_size = half_sizes.minCoeff();

  std::function<T(const Vector3<T>&)> box_sdf =
      [min_half_size](const Vector3<T>& p_WQ) {
        // Dimensionless.
        const Vector3<T> p = p_WQ / min_half_size;
        const Vector3<T> slabs_sdf = p.cwiseAbs() - Vector3<T>::Ones();
        // SDF inside the box. It is negative inside and zero outside.
        const T inside_sdf = slabs_sdf.cwiseMin(T(0)).maxCoeff();
        // SDF outside the box. It is zero inside and positive outside.
        const T outside_sdf = slabs_sdf.cwiseMax(T(0)).norm();
        return inside_sdf + outside_sdf;
      };

  std::function<Vector3<T>(const Vector3<T>&)> grad_box_sdf =
      [min_half_size](const Vector3<T>& p_WQ) {
        // Dimensionless.
        const Vector3<T> p = p_WQ / min_half_size;
        const Vector3<T> slabs_sdf = p.cwiseAbs() - Vector3<T>::Ones();
        // SDF inside the box. It is negative inside and zero outside.
        int max_index;
        const T inside_sdf = slabs_sdf.cwiseMin(T(0)).maxCoeff(&max_index);
        if (inside_sdf <= T(0)) {
          if (p[max_index] > 0)
            return Vector3<T>(Vector3<T>::Unit(max_index));
          else
            return Vector3<T>(-Vector3<T>::Unit(max_index));
        } else {
          // SDF outside the box. It is zero inside and positive outside.
          // const T outside_sdf = slabs_sdf.cwiseMax(T(0)).norm();
          const Vector3<T> x_plus = slabs_sdf.cwiseMax(T(0));
          const Vector3<T> grad = x_plus.normalized();
          return grad;
        }
        DRAKE_UNREACHABLE();
      };

  auto mesh = std::make_unique<geometry::VolumeMesh<T>>(
      geometry::internal::MakeBoxVolumeMesh<T>(box, target_edge_length));

  // Analytic pressure field and gradient.
  std::vector<T> e_m_values(mesh->vertices().size());
  std::vector<Vector3<T>> grad_e_m_values(mesh->vertices().size());
  for (geometry::VolumeVertexIndex v(0); v < mesh->num_vertices(); ++v) {
    const Vector3<T>& p_MV = mesh->vertex(v).r_MV();
    e_m_values[v] = -box_sdf(p_MV);
    grad_e_m_values[v] = -grad_box_sdf(p_MV);
  }
  auto e_m = std::make_unique<geometry::VolumeMeshFieldLinear<T, T>>(
      "Sphere Pressure Field", std::move(e_m_values), mesh.get());
  auto grad_e_m =
      std::make_unique<geometry::VolumeMeshFieldLinear<Vector3<T>, T>>(
          "Sphere Pressure Gradient Field", std::move(grad_e_m_values),
          mesh.get());
  return std::make_unique<HydroelasticField<T>>(
      std::move(mesh), std::move(e_m), std::move(grad_e_m));
}

}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
