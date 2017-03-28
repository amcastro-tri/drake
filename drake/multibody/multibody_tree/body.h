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

  const BodyTopology& get_topology() const { return topology_;}

  /// Sets the topological information for this mobilizer. Topological
  /// information includes connectivity such as inboard and outboard frames and
  /// bodies as well as degrees of freedom indexing information.
  /// This is an implementation detail. User code should never call it.
  void SetTopology(const BodyTopology& topology) {
    topology_ = topology;
  }

  const Isometry3<T>& get_pose_in_world(
      const MultibodyTreeContext<T>& context) const
  {
    // TODO(amcastro-tri): Check cache validity
    return context.get_position_kinematics().get_X_WB(topology_.body_node);
  }

  /// Returns from the current state in the @p context the position of the
  /// center of mass for this body measured and expressed in the
  /// world frame `W`.
  const Vector3<T>& get_com_W(
      const MultibodyTreeContext<T>& context) const
  {
    // TODO(amcastro-tri): Check cache validity
    return context.get_position_kinematics().get_com_W(topology_.id);
  }

  const SpatialInertia<T>& get_M_Bo_W(
      const MultibodyTreeContext<T>& context) const
  {
    return context.get_position_kinematics().get_M_Bo_W(topology_.id);
  }

  const GeneralSpatialVector<T>& get_spatial_velocity_in_world(
      const MultibodyTreeContext<T>& context) const
  {
    // TODO(amcastro-tri): Check cache validity
    return context.get_velocity_kinematics().get_V_WB(topology_.body_node);
  }

  /// Computes the center of mass of this body measured and expressed in its
  /// implicity body frame B.
  /// @param[in] context Context cotaining the state of the MultibodyTree.
  /// @returns com_B The center of mass of this body measured and expressed in
  ///                its implicity body frame `B`.
  virtual Vector3<T> CalcCenterOfMassInBodyFrame(
      const MultibodyTreeContext<T>& context) const = 0;

  /// Computes the UnitInertia of this body as measured and expressed in the
  /// body frame `B`.
  /// @param[in] context Context cotaining the state of the MultibodyTree.
  /// @returns G_Bo_B UnitInertia of this body computed about the origin `Bo`
  ///                 of its frame `B`, expressed in `B`.
  virtual UnitInertia<T> CalcUnitInertiaInBodyFrame(
      const MultibodyTreeContext<T>& context) const = 0;

  /// Computes the mass of the body for the current configuration of the
  /// @p context.
  virtual T CalcMass(const MultibodyTreeContext<T>& context) const = 0;

  void PrintTopology() const {
    std::cout << "Body id: " << topology_.id << std::endl;
    std::cout << "Body level: " << topology_.level << std::endl;
    std::cout << "Parent body: " << topology_.parent_body << std::endl;

    if (topology_.get_num_children() != 0) {
      std::cout << "Children ( " <<
                topology_.get_num_children() << "): ";
      std::cout << topology_.child_bodies[0];
      for (int ichild = 1; ichild < topology_.get_num_children(); ++ichild) {
        std::cout << ", " << topology_.child_bodies[ichild];
      }
      std::cout << std::endl;
    }

    std::cout << "Material frames (" <<
              topology_.get_num_material_frames() << "): ";
    std::cout << topology_.material_frames[0];
    for (int iframe = 1;
         iframe < topology_.get_num_material_frames(); ++iframe) {
      std::cout << ", " << topology_.material_frames[iframe];
    }
    std::cout << std::endl;

    std::cout << "Body node: " << topology_.body_node << std::endl;
  }

 protected:
  void set_body_frame(FrameIndex body_frame_id)
  {
    // Asserts that the first frame added is the body frame.
    DRAKE_ASSERT(frames_.size() == 0);
    frames_.push_back(body_frame_id);
    body_frame_id_ =  body_frame_id;
  }

 private:
  BodyTopology topology_;

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
