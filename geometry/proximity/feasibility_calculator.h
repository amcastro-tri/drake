#pragma once

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/hydroelastic_internal.h"
namespace drake {
namespace geometry {
namespace internal {
namespace hydroelastic {

using Eigen::Vector3d;

template <typename T>
class FeasibilityCalculator {
 public:
  /* Constructs the fully-specified calculator. The values are as described in
   the class documentation. Some parameters (noted below) are aliased in the
   data and must remain valid at least as long as the
   FeasibilityCalculator instance.

   @param geometries              The set of all hydroelastic geometric
                                  representations. Aliased. */
  FeasibilityCalculator(
      Geometries* geometries,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs_prev,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs_next)
      : geometries_(*geometries),
        X_WGs_prev_(*X_WGs_prev),
        X_WGs_next_(*X_WGs_next) {
    DRAKE_DEMAND(geometries != nullptr);
    DRAKE_DEMAND(X_WGs_prev != nullptr);
    DRAKE_DEMAND(X_WGs_next != nullptr);
  }

  /* Returns true if the two geometries do not collide over the
     (linearly-interpolated) trajectories of their respective mesh vertices.

     @param id_A     Id of the first object in the pair (order insignificant).
     @param id_B     Id of the second object in the pair (order insignificant).
      */
  bool IsFeasibleTrajectory(GeometryId id_A, GeometryId id_B);

 private:
  bool IsFeasibleTrajectoryVertexFace(GeometryId id_A, int v_A, GeometryId id_B,
                                      int t_B) const;

  bool IsFeasibleTrajectoryEdgeEdge(GeometryId id_A, int e_A, GeometryId id_B,
                                    int e_B) const;

  /* The hydroelastic geometric representations.  */
  Geometries& geometries_;
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs_prev_;
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs_next_;
};

}  // namespace hydroelastic
}  // namespace internal
}  // namespace geometry
}  // namespace drake
