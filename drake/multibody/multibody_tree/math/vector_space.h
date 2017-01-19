#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/common.h"

namespace drake {
namespace multibody {
namespace math {

// Primary template definition.
template <class V> struct VectorSpace;

// Specialization to Vector3
template <typename T>
struct VectorSpace<Vector3<T>> {
  enum {
    kNumDimensions = 3,
    kSpatialVectorSize = 6,
    kSpatialVectorAngularSize = 3,
    kSpatialVectorLinearSize = 3
  };
  typedef T ScalarType;
  typedef Vector3<T> Spin;  // Type of the angular quantities.
  typedef Vector3<T> Vec;  // Type of linear quantities.
  typedef SpatialVector<Vec> SpatialVec;
};

// Specialization to Vector2
template <typename T>
struct VectorSpace<Vector2<T>> {
  enum {
    kNumDimensions = 2,
    kSpatialVectorSize = 3,
    kSpatialVectorAngularSize = 1,
    kSpatialVectorLinearSize = 2
  };
  typedef T ScalarType;
  typedef Vector1<T> Spin;
  typedef Vector2<T> Vec;
};

// Specialization to Vector2 with options (Exg: Alignment)
template <typename T, int Options>
struct VectorSpace<Eigen::Matrix<T, 2, 1, Options>> {
  enum {
    kNumDimensions = 2,
    kSpatialVectorSize = 3,
    kSpatialVectorAngularSize = 1,
    kSpatialVectorLinearSize = 2
  };
  typedef T ScalarType;
  typedef Eigen::Matrix<T, 1, 1, Options> Spin;
  typedef Eigen::Matrix<T, 2, 1, Options> Vec;
  typedef SpatialVector<Vec> SpatialVec;
};

template <typename T> using Spin3 = typename VectorSpace<Vector3<T>>::Spin;
template <typename T> using Spin2 = typename VectorSpace<Vector2<T>>::Spin;

}  // namespace math
}  // namespace multibody
}  // namespace drake
