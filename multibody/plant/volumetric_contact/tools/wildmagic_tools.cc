#include "drake/multibody/plant/volumetric_contact/tools/wildmagic_tools.h"

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"


namespace drake {
namespace multibody {

template <typename T>
void ReExpressConvexPolyhedron(const Wm5::ConvexPolyhedron<T>& mesh_B,
                               const Isometry3<T>& X_AB,
                               Wm5::ConvexPolyhedron<T>* mesh_A_ptr) {  
  DRAKE_DEMAND(mesh_A_ptr != nullptr);
  Wm5::ConvexPolyhedron<T>& mesh_A = *mesh_A_ptr;
  // I found out that mesh_A = mesh_B below somehow destroys both mesh A and B.
  // I do not know why, so I'll just demand that A and B are two different
  // meshes in memory.
  DRAKE_DEMAND(mesh_A_ptr != &mesh_B);
  mesh_A = mesh_B;  // "allocate" sime size.
  for (int i = 0; i < mesh_A.GetNumVertices(); ++i) {
    const Wm5::Vector3<double> p_AP =
        ToWm5Vector3(X_AB * ToEigenVector3(mesh_B.Point(i)));
    mesh_A.Point(i) = p_AP;
  }
  mesh_A.UpdatePlanes();
}

template void ReExpressConvexPolyhedron(const Wm5::ConvexPolyhedron<double>& mesh_B,
                                        const Isometry3<double>& X_AB,
                                        Wm5::ConvexPolyhedron<double>* mesh_A_ptr);

}  // namespace multibody
}  // namespace drake


