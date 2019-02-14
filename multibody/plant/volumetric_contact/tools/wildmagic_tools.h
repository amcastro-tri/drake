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

/// Re-expresses mesh B in another frame A. 
/// @pre mesh_A and mesh_B are two different in-memory meshes. That is, this
/// method aborts if their address is the same. mesh_A_ptr might point to an
/// empty mesh, that's ok.
template <typename T>
void ReExpressConvexPolyhedron(const Wm5::ConvexPolyhedron<T>& mesh_B,
                               const Isometry3<T>& X_AB,
                               Wm5::ConvexPolyhedron<T>* mesh_A_ptr);

}  // namespace multibody
}  // namespace drake


