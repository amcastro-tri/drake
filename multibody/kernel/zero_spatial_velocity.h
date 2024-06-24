#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/multibody/math/spatial_algebra.h"

namespace drake {
namespace multibody {
namespace internal {

class ZeroSpatialMotion {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ZeroSpatialMotion);

  // Zero spatial velocity.
  ZeroSpatialMotion() = default;

 private:
  template <template <typename> class SpatialMotion, typename T>
  friend SpatialMotion<T> operator+(const SpatialMotion<T>& V_WPb,
                                    const ZeroSpatialMotion);
};

// Trivial composition with a zero spatial velocity.
// @returns V_WPb, always.
template <template <typename> class SpatialMotion, typename T>
SpatialMotion<T> operator+(const SpatialMotion<T>& V_WPb,
                           const ZeroSpatialMotion) {
  static_assert(std::is_same_v<SpatialMotion<T>, SpatialVelocity<T>> ||
                std::is_same_v<SpatialMotion<T>, SpatialAcceleration<T>>);
  return V_WPb;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
