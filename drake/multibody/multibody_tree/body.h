#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyTree;
template<typename T> class BodyFrame;

template <typename T>
class Body : public MultibodyTreeElement<Body<T>, BodyIndex> {
 public:
  virtual int get_num_positions() const = 0;

  virtual int get_num_velocities() const = 0;

  FrameIndex get_body_frame_id() const { return body_frame_id_;}

  virtual void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) = 0;

  /// @returns The number of material frames attached to this body.
  int get_num_frames() const { return static_cast<int>(frames_.size());}

 protected:
  void set_body_frame(FrameIndex body_frame_id)
  {
    // Asserts that the first frame added is the body frame.
    DRAKE_ASSERT(frames_.size() == 0);
    frames_.push_back(body_frame_id);
    body_frame_id_ =  body_frame_id;
  }

 private:
  FrameIndex body_frame_id_;
  std::vector<FrameIndex> frames_;

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

  //const BodyTopology& get_topology() const { return topology_; };

  // Attorney-Client Idiom to allow MultibodyTree<T> to modify private topology
  // information in Body<T>.
  // see: http://stackoverflow.com/questions/3217390/clean-c-granular-friend-equivalent-answer-attorney-client-idiom/
  // This class needs to be public so that its friends methods can access it
  // from within Body. However, even if public, its only useful to its friend
  // methods and therefore it's safe to have it here.
  //class PrivateAccessAttorney;
};

}  // namespace multibody
}  // namespace drake
