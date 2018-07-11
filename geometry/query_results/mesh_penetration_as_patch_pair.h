#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

// Forward declaration
// For now, assume that there exists a triangle primitive in the mesh. It
// supports various operations of use to the patch.
class Triangle;

/** @name     Mesh penetration reported in pairs of patches

 A unique characterization of the intersection of two penetrating meshes. The
 characterization consists of a pair of _patches_. The patch is implicitly
 defined by two sets of "decorated" points. Each set is comprised of points
 on the surface of one of the penetrating meshes.

 Consider two penetrating surfaces A and B. We define sets of points Aᵢ and
 Bⱼ, points that lie on the surface of meshes A and B, respectively. Each
 point is associated with a collection of correlated quantities. We'll describe
 these quantities with respect to a particular point in the set for A, A₀, with
 the understanding that it extends to all other points in the set for A and is
 reflected in the set for B.

 For point Aᵢ we have the following quantities.
   - φᵢ, the signed distance from Aᵢ to the surface of mesh B in meters
     (positive indicates *outside* the other mesh, non-positive indicates
     touching or inside),
   - The closest point on B to Aᵢ (called Bᴬᵢ). It should be the case that
     `|Aᵢ - Bᴬᵢ| = |φᵢ|`.
   - A triangle on which point Aᵢ lies, called Tᴬᵢ.
   - The barycentric coordinates for Aᵢ with respect to triangle Tᴬᵢ: αᴬᵢ, βᴬᵢ,
     γᴬᵢ. If Aᵢ is a node of mesh A, then one of αᴬᵢ, βᴬᵢ, γᴬᵢ will be one
     and the other values will be zero. In any case, they should sum to zero.
   - A triangle on which point Bᴬᵢ lies, called Tᴮᴬᵢ.
   - The barycentric coordinates for Bᴬ₀ with respect to triangle Tᴮᴬᵢ: αᴮᴬᵢ,
     βᴮᴬᵢ, γᴮᴬᵢ.
   - The outward-pointing unit normal n̂ᴬᵢ of mesh A at point Aᵢ. Generally,
     `(Bᴬᵢ - Aᵢ) · n̂ᴬᵢ ≠ φᵢ`.
   - The outward-pointing unit normal n̂ᴮᴬᵢ of mesh B at point Bᴬᵢ.

 Note: this representation does not *explicitly* encode the connectivity of the
 patch. However, the connectivity can be inferred by exploring the triangles.

 <!--
    General questions (and, optionally, possible answers):
      1. What is the definition of the normal of the mesh at a node?
        - possible answer 1: the area-weighted average of the incident faces.
        - possible answer 2: defined explicitly by the mesh from arbitrary
          criteria.
      2. What frames should these quantities be measured/expressed in?

    General assumptions:
      1. This struct can *only* be populated as a result of colliding two
         meshes. Attempting to do so with *any* other kind of geometric
         primitive should throw an exception.

    General notes:
      1. In order to support edge-edge penetration *and* to produce nodes,
         virtual node(s) must be created which don't correlate with mesh
         vertices. (I.e., Aᵢ may *not* be a "node" and no information about
         which triangle it belongs to will be given.) In this case, it seems
         reasonable, that Aᵢ should *also* map to a Triangle and a set of
         barycentric coordinates.
 -->
 */

//@{

/** The data for a single patch point (and its correlated quantities). The
 patch point doesn't specifically track which geometries these quantities are
 derived from. It relies on the greater context to provide that information.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.  */
template <typename T>
struct PenetrationPatchPoint {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PenetrationPatchPoint)

  /** A position vector from the world origin to a patch point on the surface of
   A, measured and expressed in the world frame. */
  const Vector3<T> p_WoA_W;

  /** A position vector from the world origin to a point on mesh B that is
   closest to the patch point (`p_WoA_W`) on surface A. */
  const Vector3<T> p_WoBA_W;

  /** A reference to the triangle on A to which the patch point belongs. Its
   lifespan is that of the underlying mesh. */
  const Triangle& triangle_A;

  /** The barycentric coordinates of the patch point on triangle_A.  */
  const Vector3<T> barycentric_A;

  /** A reference to the triangle of mesh B on which the the point nearest the
   patch point lies.  */
  const Triangle& triangle_B;

  /** The barycentric coordinates of the nearest point on B on triangle_B.  */
  const Vector3<T> barycentric_B;

  /** The outward facing normal of mesh A at the patch point, measured and
   expressed in the world frame.  */
  const Vector3<T> normal_A_W;

  /** The outward facing normal of mesh B at the patch point's nearest point on
   B, measured and expressed in the world frame.  */
  const Vector3<T> normal_B_W;
};

/** The characterization of penetration between two *meshes* as a pair of
 surface patches. Provides the identifiers for the two penetrating mesh
 geometries and the patch points that lie on each meshes' surfaces.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar.  */
template <typename T>
struct MeshPenetrationAsPatchPair {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshPenetrationAsPatchPair)

  /** The identifier for geometry A. */
  const GeometryId id_A;

  /** The identifier for geometry B. */
  const GeometryId id_B;

  /** The collection of patch points on the surface of geometry A.  */
  const std::vector<PenetrationPatchPoint> points_A;

  /** The collection of patch points on the surface of geometry B.  */
  const std::vector<PenetrationPatchPoint> points_B;
};

// @}

}  // namespace geometry
}  // namespace drake
