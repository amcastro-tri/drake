#pragma once

#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

/// This struct stores a contact Jacobians computation when a point contact
/// model is used.
/// At a given state of the multibody system, there will be `nc` contact pairs.
/// For each penetration pair involving bodies A and B a contact frame C is
/// defined by the rotation matrix `R_WC = [Cx_W, Cy_W, Cz_W]` where
/// `Cz_W = nhat_BA_W` equals the normal vector pointing from body B into body
/// A, expressed in the world frame W. See PenetrationAsPointPair for further
/// details on the definition of each contact pair. Versors `Cx_W` and `Cy_W`
/// constitute a basis of the plane normal to `Cz_W` and are arbitrarily chosen.
/// Bellow, v denotes the vector of generalized velocities, of size `nv`.
/// @see MultibodyPlant::EvalContactJacobians().
template <class T>
struct ContactJacobians {
  /// Normal contact Jacobian.
  /// `Jn` is a matrix of size `nc x nv` such that `vn = Jn⋅v` is the separation
  /// speed for each contact point, defined to be positive when bodies are
  /// moving away from each other.
  MatrixX<T> Jn;

  /// Tangential contact forces Jacobian.
  /// `Jt` is a matrix of size `2⋅nc x nv` such that `vt = Jt⋅v` concatenates
  /// the two-dimensional relative tangential velocity vector `vx_AcBc_C` for
  /// each contact pair.
  MatrixX<T> Jt;

  /// List of contact frames orientation R_WC in the world frame W for each
  /// contact pair.
  std::vector<Matrix3<T>> R_WC_list;
};

}  // namespace multibody
}  // namespace drake
