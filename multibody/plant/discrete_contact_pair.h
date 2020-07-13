#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace multibody {

/** A characterization of the intersection of two penetrating geometries with
 the only information a discrete solver cares about.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
struct DiscreteContactPair {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiscreteContactPair)
  DiscreteContactPair() = default;

  /** The id of the first geometry in the contact. */
  geometry::GeometryId id_A;
  /** The id of the second geometry in the contact. */
  geometry::GeometryId id_B;
  /** Position of contact point C in W. **/
  Vector3<T> p_WC;
  /** The unit-length normal which defines the penetration direction, pointing
   from geometry B into geometry A, measured and expressed in the world frame.
   It _approximates_ the normal to the plane on which the contact patch lies. */
  Vector3<T> nhat_BA_W;
  /** The normal contact force at the t0 configuration. */
  T fn0{0.0};
  /** The effective discrete stiffness. **/
  T stiffness{0.0};
  T damping{0.0};
};

}  // namespace multibody
}  // namespace drake
