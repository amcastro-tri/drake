#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/vector_space.h"

namespace drake {
namespace multibody {
namespace math {

// V3 x V3 ∈ V3
template <typename T>
Vector3<T> cross(const Vector3<T>& a, const Vector3<T>& b) {
  return a.cross(b);
}

// V2 x V2 ∈ V1
template <typename T>
Vector1<T> cross(const Vector2<T>& a, const Vector2<T>& b) {
  return Vector1<T>(a.x() * b.y() - a.y() * b.x());
}

// V1 x V2 ∈ V2
template <typename T>
Vector2<T> cross(const Vector1<T>& a, const Vector2<T>& b) {
  return a.x() * Vector2<T>(-b.y(), b.x());
}

// V2 x V1 ∈ V2
template <typename T>
Vector2<T> cross(const Vector2<T>& a, const Vector1<T>& b) {
  return -cross(b, a);
}

// V1 x V1 ∈ V1 is a null operation.
// The analogy comes from the 3D conservation of angular momentum for a rigid
// body:
// dL/dt = I * wdot + w x L
// Where dLdt is an angular quantity (Vector1<T>) and therefor the two terms on
// the right hand side also need to be an angular quantity. However, since w
// and L in 2D are aligned, their cross product is zero.
template <typename T>
Vector1<T> cross(const Vector1<T>& a, const Vector1<T>& b) {
  return Vector1<T>::Zero();
}

// Cross product skew symmetric matrix in 3D.
template<typename T>
Matrix3<T> CrossProductMatrix(const Vector3<T>& v) {
  return (Matrix3<T>() <<
                      T(0), -v[2],  v[1],
                      v[2],  T(0), -v[0],
                     -v[1],  v[0],  T(0)).finished();
}

// Cross product skew symmetric matrix in 2D.
template<typename T>
RowVector2<T> CrossProductMatrix(const Vector2<T>& v) {
  return RowVector2<T>(-v.y(), v.x());
}

}  // namespace math
}  // namespace multibody
}  // namespace drake
