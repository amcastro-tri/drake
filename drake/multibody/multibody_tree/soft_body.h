#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/mass_properties.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyTree;
template<typename T> class SoftBodyFrame;
template<typename T> class FixedOffsetFrame;

template <typename T>
class SoftBody : public Body<T> {
 public:
  /// Rigidly attaches a frame `F` to one of the material frames `M` of this
  /// body.
  /// @param[in] X_MF The pose of the frame `F` in the material frame `M`.
  FixedOffsetFrame<T>& AttachFixedOffsetFrame(
      const SoftBodyFrame<T>& M, const Isometry3<T>& X_MF);

  // No-op for rigid bodies since all frames attached to them are rigidly
  // attached. The poses X_BF of frames F attached to body a body frame B are
  // constant.
  void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) {
    PositionKinematicsCache<T>* pc = context.get_mutable_position_kinematics();
    // Range of frames transforms X_BM that belong to this body.
    auto first_material_frame_iterator = pc->get_material_frames_pool().begin();
    auto last_material_frame_iterator = pc->get_material_frames_pool().end();
    
    // Two kind of frames can be attached to a soft body:
    //   1. A SoftBodyFrame.
    //   2. A FixedOffsetFrame attached to either the body reference frame or
    //      to any of its SoftBodyFrame's.
    // These must be updated in that order.

    // Update SoftBodyFrame's. These are attached to particular material
    // points in the body.
    CalcSoftBodyFrames(
        context, first_material_frame_iterator, last_material_frame_iterator);

    // Update Other FixedOffsetFrame's that might be attached to the body.
    // In general for a fixed offset frame F attached to a material frame
    // M on the body, its pose is computed as: X_BF(qf_B) = X_BM(qf_B) * X_MF
    // where X_MB was updated above by CalcSoftBodyFrames() and X_MF is the
    // fixed posed of frame F in M, which is encoded in FixedOffsetFrame.
    for (const auto& frame_pair: fixed_offset_frames_) {
      const FrameIndex parent_frame = frame_pair.first;  // SoftBodyFrame.
      const FrameIndex child_frame = frame_pair.second;  // FixedOffsetFrame.
      auto& X_BM_pool = pc->get_material_frames_pool();
      const Isometry3<T>& X_BM = X_BM_pool[parent_frame];  // Maybe another index?
      const Isometry3<T>& X_MF =
          get_parent_tree().get_frame(child_frame).get_X_MF();
      Isometry3<T>& X_BF = X_BM_pool[child_frame];  // Maybe another index?
      X_BF = X_BM * X_BF;
    }
  }

  /// Computes the pose `X_BM(qf_B)` of each SoftBodyFrame `M` attached to
  /// this body measured and expressed in the frame of the body `B`. Where we
  /// have denotes with `qf_B` the flexible degrees of freedom corresponding to
  /// this body.
  /// param[in]  context               The context of the parent MultibodyTree.
  /// param[out] material_frames_poses An array for which each entry corresponds
  ///            to the pose `X_BM` of a SoftBodyFrame in this body.
  virtual void CalcSoftBodyFrames(
      const MultibodyTreeContext<T>& context,
      std::vector<Isometry3<T>>::iterator first_material_frame,
      std::vector<Isometry3<T>>::iterator last_material_frame) = 0;

  /// Computes the rigid body inertia matrix for a given, fixed, value of the
  /// flexible generalized coordinates @p qf.
  //virtual SpatialMatrix DoCalcSpatialInertia(const VectorX<T>& qf) const = 0;

  /// Computes the total mass matrix of this body including the rigid dof's
  /// block, the flexible dof's block and the off-diagonal blocks coupling rigid
  /// and flexible modes.
  /// The default implementation assumes M is the rigid body spatial inertia
  /// computed with DoCalcSpatialInertia().
  //virtual void DoCalcMassMatrix(const VectorX<T>& qf, MatrixX<T>* M) const {
  //  *M = DoCalcSpatialInertia(qf);
  //}

  //const SoftBodyTopology& get_topology() const { return topology_; };

  // Attorney-Client Idiom to allow MultibodyTree<T> to modify private topology
  // information in SoftBody<T>.
  // see: http://stackoverflow.com/questions/3217390/clean-c-granular-friend-equivalent-answer-attorney-client-idiom/
  // This class needs to be public so that its friends methods can access it
  // from within SoftBody. However, even if public, its only useful to its friend
  // methods and therefore it's safe to have it here.
  //class PrivateAccessAttorney;

 private:
  // List of SoftBodyFrame's registered for this body.
  std::vector<FrameIndex> soft_body_frames_;

  // Vector of Pairs <parent_soft_frame_id, fixed_offset_frame>.
  // Where parent_soft_frame_id is a frame identifier that must be present in
  // soft_body_frames_ (this is checked when adding frames).
  std::vector<FrameIndex, FrameIndex> fixed_offset_frames_;
};

}  // namespace multibody
}  // namespace drake
