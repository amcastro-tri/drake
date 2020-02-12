#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/body_node.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

/// For internal use only of the MultibodyTree implementation.
/// While all code that is common to any node can be placed in the BodyNode
/// class, %BodyNodeImpl provides compile-time fixed-size BodyNode
/// implementations so that all operations can be performed with fixed-size
/// stack-allocated Eigen variables.
/// In particular, most of the across mobilizer code for velocity kinematics
/// lives in this class since the across mobilizer hinge matrices `H_FM(q)`
/// (defined such that the across mobilizer spatial velocity relates to the
/// mobilizer's generalized velocities v by `V_FM = H_FM(q) * v`) have a
/// compile-time fixed size. For a more detailed discussion of the role of a
/// BodyNode in a MultibodyTree refer to the class documentation for BodyNode.
template <typename T, int  num_positions, int num_velocities>
class BodyNodeImpl : public BodyNode<T> {
 public:
  // static constexpr int i = 42; discouraged.  See answer in:
  // http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = num_positions, nv = num_velocities};

  /// Given a body and its inboard mobilizer in a MultibodyTree this constructor
  /// creates the corresponding %BodyNode. See the BodyNode class documentation
  /// for details on how a BodyNode is defined.
  /// @param[in] parent_node
  ///   A const pointer to the parent BodyNode object in the tree structure of
  ///   the owning MultibodyTree. It can be a `nullptr` only when `body` **is**
  ///   the **world** body, otherwise the parent class constructor will abort.
  /// @param[in] body The body B associated with `this` node.
  /// @param[in] mobilizer The mobilizer associated with this `node`. It can
  ///                      only be a `nullptr` for the **world** body.
  BodyNodeImpl(const internal::BodyNode<T>* parent_node,
               const Body<T>* body, const Mobilizer<T>* mobilizer) :
      BodyNode<T>(parent_node, body, mobilizer) {}

  void CalcArticulatedBodyAccelerations_BaseToTip(
      const systems::Context<T>& /* context */,
      const PositionKinematicsCache<T>& pc,
      const ArticulatedBodyInertiaCache<T>& abic,
      const ArticulatedBodyForceBiasCache<T>& aba_force_bias_cache,
      const Eigen::Ref<const MatrixUpTo6<T>>& H_PB_W,
      const SpatialAcceleration<T>& Ab_WB,
      AccelerationKinematicsCache<T>* ac) const final {
    DRAKE_THROW_UNLESS(ac != nullptr);

    // As a guideline for developers, please refer to @ref
    // abi_computing_accelerations for a detailed description of the algorithm
    // and the notation in use.

    // Get the spatial acceleration of the parent.
    const SpatialAcceleration<T>& A_WP = parent_node_->get_A_WB(*ac);

    // Shift vector p_PoBo_W from the parent origin to the body origin.
    const Vector3<T>& p_PoBo_W = get_p_PoBo_W(pc);

    // Rigidly shift the acceleration of the parent node.
    const SpatialAcceleration<T> Aplus_WB = SpatialAcceleration<T>(
        A_WP.rotational(),
        A_WP.translational() + A_WP.rotational().cross(p_PoBo_W));

    SpatialAcceleration<T>& A_WB = get_mutable_A_WB(ac);
    A_WB = Aplus_WB + Ab_WB;

    // These quantities do not contribute when nv = 0. We skip them since Eigen
    // does not allow certain operations on zero-sized objects.
    if constexpr (nv != 0) {      
      // Compute nu_B, the articulated body inertia innovations generalized
      // acceleration.
      Vector<T, nv> nu_B;
      if constexpr (nv == 1) {
        // The inverse of D is simply the scalar Di.
        const T Di = 1.0 / get_ldlt_D_B(abic).vectorD()(0);
        nu_B = Di * get_e_B(aba_force_bias_cache);
      } else {
        nu_B = get_ldlt_D_B(abic).solve(get_e_B(aba_force_bias_cache));
      }

      // Mutable reference to the generalized acceleration.
      auto vmdot = get_mutable_accelerations(ac).template head<nv>();
      const auto g_PB_W = get_g_PB_W(abic).template topLeftCorner<6, nv>();
      vmdot = nu_B - g_PB_W.transpose() * A_WB.get_coeffs();

      // Update with vmdot term the spatial acceleration of the current body.
      A_WB += SpatialAcceleration<T>(H_PB_W * vmdot);
    }
  }    

  private:
   using BodyNode<T>::get_p_PoBo_W;
   using BodyNode<T>::get_num_mobilizer_velocities;
   using BodyNode<T>::get_mutable_A_WB;
   using BodyNode<T>::get_g_PB_W;
   using BodyNode<T>::get_mutable_accelerations;
   using BodyNode<T>::get_ldlt_D_B;
   using BodyNode<T>::get_e_B;
   using BodyNode<T>::parent_node_;

   // TODO(amcastro-tri): Implement methods for computing velocity kinematics
   // using fixed-size Eigen matrices.
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
