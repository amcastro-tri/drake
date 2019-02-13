#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"


namespace drake {
namespace multibody {

template <typename T>
Vector3<T> ToEigenVector3(const Wm5::Vector3<T>& p) {
  return Vector3<T>(p.X(), p.Y(), p.Z());
}

template <typename T>
Wm5::Vector3<T> ToWm5Vector3(const Vector3<T>& p) {
  return Wm5::Vector3<T>(p.x(), p.y(), p.z());
}

template <typename T>
void ReExpressConvexPolyhedron(const Wm5::ConvexPolyhedron<T>& mesh_B,
                               const Isometry3<T>& X_AB,
                               Wm5::ConvexPolyhedron<T>* mesh_A_ptr) {
  Wm5::ConvexPolyhedron<T>& mesh_A = *mesh_A_ptr;
  mesh_A = mesh_B;  // "allocate" sime size.
  for (int i = 0; i < mesh_A.GetNumVertices(); ++i) {
    const Wm5::Vector3<double> p_AP =
        ToWm5Vector3(X_AB * ToEigenVector3(mesh_B.Point(i)));
    mesh_A.Point(i) = p_AP;
  }
  mesh_A.UpdatePlanes();
}

}  // namespace multibody
}  // namespace drake


