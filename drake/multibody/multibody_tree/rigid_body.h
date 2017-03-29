#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
//#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mass_properties.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "multibody_tree.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyTree;
template<typename T> class RigidBodyFrame;

template <typename T>
class RigidBody : public Body<T> {
 public:
  /// Rigidly attaches a frame `F` to this body.
  /// @param[in] X_BF The pose of the frame `F` in the frame of the body `B`.
  //RigidBodyFrame<T>& RigidlyAttachFrame(const Isometry3<T>& X_BF);

  int get_num_flexible_positions() const final { return 0; }

  int get_num_flexible_velocities() const final { return 0; }

  // No-op for rigid bodies since all frames attached to them are rigidly
  // attached. The poses X_BF of frames F attached to body a body frame B are
  // constant.
  void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) final {}

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

  //const RigidBodyTopology& get_topology() const { return topology_; };

  // Attorney-Client Idiom to allow MultibodyTree<T> to modify private topology
  // information in RigidBody<T>.
  // see: http://stackoverflow.com/questions/3217390/clean-c-granular-friend-equivalent-answer-attorney-client-idiom/
  // This class needs to be public so that its friends methods can access it
  // from within RigidBody. However, even if public, its only useful to its friend
  // methods and therefore it's safe to have it here.
  //class PrivateAccessAttorney;

  /// Creates a new %RigidBody and adds it to the %MultibodyTree world.
  /// The MultibodyTree @p tree takes ownership of the newly created body.
  /// @param[in] tree The parent MultibodyTree to which this body will be added.
  /// @param[in] mass_properties Default mass properties for this rigid body.
  /// @returns A reference to the newly created rigid body.
  static RigidBody<T>& Create(MultibodyTree<T>* tree,
                              const MassProperties<T>& mass_properties);

  /// Returns the default center of mass of this body measured and expressed in
  /// its implicity body frame `B`. The returned center of mass is independent
  /// of the input @p context.
  Vector3<T> CalcCenterOfMassInBodyFrame(
      const MultibodyTreeContext<T>& context) const final {
    return default_mass_properties_.get_com();
  }

  /// Returns the default UnitInertia of this body as measured and expressed in
  /// the body frame `B`. The returned unit inertia is independent of the input
  /// @p context.
  /// @param[in] context Context cotaining the state of the MultibodyTree.
  /// @returns G_Bo_B UnitInertia of this body computed about the origin `Bo`
  ///                 of its frame `B`, expressed in `B`.
  UnitInertia<T> CalcUnitInertiaInBodyFrame(
      const MultibodyTreeContext<T>& context) const final {
    return default_mass_properties_.get_unit_inertia();
  }

  /// Returns the default mass of the body.
  T CalcMass(const MultibodyTreeContext<T>& context) const final {
    return default_mass_properties_.get_mass();
  }

  void Compile() final {};

 private:
  MassProperties<double> default_mass_properties_;

  /// Creates a new body with the provided mass properties. This body must
  /// immediately be added to a MultibodyTree.
  RigidBody(const MassProperties<double>& mass_properties);

  // List of frames rigidly attached to this body.
  //std::vector<FrameIndex> material_frames_;
};

}  // namespace multibody
}  // namespace drake
