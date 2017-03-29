#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace math {

namespace internal {
template <class T>
struct traits { };
}

// Forward declarations.
template <class V> struct VectorSpace;
template <class V> class SpatialVector;
template <class V> class SpatialInertia;

}  // namespace math
}  // namespace multibody
}  // namespace drake
