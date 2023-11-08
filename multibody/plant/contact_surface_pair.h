#pragma once

#include <optional>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
struct ContactSurfacePair {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurfacePair);    

  // Struct to store the block contribution from a given tree to the contact
  // Jacobian for a contact pair.
  struct JacobianTreeBlock {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JacobianTreeBlock);

    JacobianTreeBlock(TreeIndex tree_in,
                      contact_solvers::internal::MatrixBlock<T> J_in)
        : tree(tree_in), J(std::move(J_in)) {}

    // Index of the tree for this block.
    TreeIndex tree;

    // J.cols() must equal the number of generalized velocities for
    // the corresponding tree.
    contact_solvers::internal::MatrixBlock<T> J;
  };

  // Per face data.
  struct FaceData {
    T phi0{0.0};
    T stiffness{0.0};
    // Position of the face center Q wrt centroid S, expressed in contact
    // frame C.
    Vector3<T> p_SQ_C;
    // Orientation of contact frame C for this face.
    math::RotationMatrix<T> R_WC;
  };

  // Data pertaining the entire surface.
  struct SurfaceData {
    /* Dissipation and friction are the same across the entire contact surface.
     */
    T damping{0.0};
    T dissipation_time_scale{NAN};
    T friction_coefficient{NAN};

    // Data for each face element.
    std::vector<FaceData> face_data;
  };

  /* Index of the original contact surface. */
  int surface_index;

  /* The id of the first geometry in the contact. */
  geometry::GeometryId id_A;
  int object_A;
  // TODO: Add p_AoS_W for multibody forces reporting.

  /* The id of the second geometry in the contact. */
  geometry::GeometryId id_B;
  int object_B;
  // TODO: Add p_BoS_W for multibody forces reporting.

  /* Jacobian for the relative spatial velocity V_AsBs_W of centroid S of the
  contact surface. */
  std::vector<JacobianTreeBlock> jacobian;

  SurfaceData surface_data;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
