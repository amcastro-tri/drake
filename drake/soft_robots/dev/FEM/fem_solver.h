#pragma once

#include <functional>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/soft_robots/dev/FEM/mesh.h"
#include "drake/soft_robots/dev/FEM/triangle_element.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {

template <typename T>
using ScalarFieldFunction =
std::function<T(const Eigen::Ref<const MatrixX<T>>& x)>;

template <typename T>
class FEMSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FEMSolver);

  FEMSolver(const Mesh<T>* mesh) : reference_mesh_(mesh) {
    if (mesh == nullptr)
      throw std::runtime_error("Mesh is a nullptr.");
  }

  void ComputeIntegralOfScalarField(ScalarFieldFunction<T> field,
    T* function_integral, T* volume) const {
    const int num_elements = reference_mesh_->get_num_elements();
    //const int num_nodes = reference_mesh_->get_num_nodes();
    const int nen = reference_mesh_->get_num_element_nodes();
    const int npd = reference_mesh_->get_num_physical_dimensions();
    TriangleElement<T, 3> element;

    // A one-point Gauss quadrature for a triangle.
    const int ngp = 1;
    Vector2<T> xquad(1.0 / 3.0, 1.0 / 3.0);

    // Element nodes.
    MatrixX<T> xa(npd, nen);

    // Jacobian at the quadrature point.
    Vector1<T> jac;

    // Position at the Gauss point.
    Vector3<T> xgp;

    *volume = 0;
    *function_integral = 0;

    for (int iel = 0; iel < num_elements; ++iel) {
      reference_mesh_->GetElementNodes(iel, xa);
      element.CalcJacobianNorm(xa, xquad, jac);
      element.MapCoordinatesFromReferenceToPhysical(xa, xquad, xgp);

      // Loop over the Gauss points.
      for (int igp = 0; igp < ngp; ++igp) {
        const T f_gp = field(xgp.col(igp));
        *volume += jac(igp);
        *function_integral += f_gp * jac(igp);
      }
    }  // End loop over elements.

    // Assembly would go here.
    //auto element_nodes = reference_mesh_->get_element_connectivities(iel);
  }

  T ComputeVolume() const {
    const int num_elements = reference_mesh_->get_num_elements();
    //const int num_nodes = reference_mesh_->get_num_nodes();
    const int nen = reference_mesh_->get_num_element_nodes();
    const int npd = reference_mesh_->get_num_physical_dimensions();
    TriangleElement<T, 3> element;

    // A one-point Gauss quadrature for a triangle.
    const int ngp = 1;
    Vector2<T> xquad(1.0 / 3.0, 1.0 / 3.0);

    // Element nodes.
    MatrixX<T> xa(npd, nen);

    // Jacobian at the quadrature point.
    Vector1<T> jac;

    // Position at the Gauss point.
    Vector3<T> xgp;

    T volume = 0;

    for (int iel = 0; iel < num_elements; ++iel) {
      reference_mesh_->GetElementNodes(iel, xa);
      element.CalcJacobianNorm(xa, xquad, jac);
      //element.MapCoordinatesFromReferenceToPhysical(xa, xquad, xgp);

      // Loop over the Gauss points.
      for (int igp = 0; igp < ngp; ++igp) {
        volume += jac(igp);
      }
    }  // End loop over elements.

    // Assembly would go here.
    //auto element_nodes = reference_mesh_->get_element_connectivities(iel);
    return volume;
  }

 private:
  // Mesh in the reference configuration.
  // The solver does not need to update it during computation.
  const Mesh<T>* reference_mesh_;
};

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake