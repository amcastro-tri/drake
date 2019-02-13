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

}  // namespace multibody
}  // namespace drake


