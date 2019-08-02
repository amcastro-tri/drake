#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/shape_specification.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/multibody/hydroelastics/hydroelastic_field.h"
#include "drake/multibody/hydroelastics/write_meshes.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;


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
std::unique_ptr<HydroelasticField<T>> MakeCylinderHydroelasticField(
    const geometry::Cylinder& cylinder, int refinement_level) {
  const double radius = cylinder.get_radius();
  const double half_length = cylinder.get_length() / 2.0;

  // Cylinder centered at the origin with its axis along the z axis.
  std::function<T(const Vector3<T>&)> cylinder_sdf =
      [radius, half_length](const Vector3<T>& p_WQ) {
        // from: http://mercury.sexy/hg_sdf/. fCylinder().
        const Vector2<T> p_xy(p_WQ[0], p_WQ[1]);
        const T sdf_cylinder_shell = p_xy.norm() - radius;
        using std::abs;
        using std::max;
        const T sdf_sides = abs(p_WQ[2]) - half_length;
        return max(sdf_cylinder_shell, sdf_sides);
      };

  std::function<Vector3<T>(const Vector3<T>&)> grad_cylinder_sdf =
      [radius, half_length](const Vector3<T>& p_WQ) {
        const Vector2<T> p_xy(p_WQ[0], p_WQ[1]);
        const T sdf_cylinder_shell = p_xy.norm() - radius;
        using std::abs;
        using std::max;
        const T sdf_sides = abs(p_WQ[2]) - half_length;

        if (sdf_cylinder_shell > sdf_sides) {
          const Vector2<T> n_xy = p_xy.normalized();
          return Vector3<T>(n_xy[0], n_xy[1], 0.0);
        } else {
          if (p_WQ[2] > 0.0)
            return Vector3<T>(0., 0., 1.);
          else
            return Vector3<T>(0., 0., -1.);
        }
        DRAKE_UNREACHABLE();
      };

  auto tmp = geometry::internal::MakeCylinderMesh<T>(
          cylinder.get_length(), cylinder.get_radius(), refinement_level);
  vtkio::write_vtk_mesh("MakeCylinderHydroelasticField0.vtk", tmp);        

  auto mesh = std::make_unique<geometry::VolumeMesh<T>>(std::move(tmp));

  vtkio::write_vtk_mesh("MakeCylinderHydroelasticField.vtk", *mesh);

  PRINT_VAR(refinement_level);
  PRINT_VAR(mesh->num_vertices());
  PRINT_VAR(mesh->num_elements());

  // Analytic pressure field and gradient.
  std::vector<T> e_m_values(mesh->vertices().size());
  std::vector<Vector3<T>> grad_e_m_values(mesh->vertices().size());
  for (geometry::VolumeVertexIndex v(0); v < mesh->num_vertices(); ++v) {
    const Vector3<T>& p_MV = mesh->vertex(v).r_MV();
    e_m_values[v] = -cylinder_sdf(p_MV);
    grad_e_m_values[v] = -grad_cylinder_sdf(p_MV);
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
