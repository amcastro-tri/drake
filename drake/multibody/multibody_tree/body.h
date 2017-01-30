#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/mass_properties.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_member.h"
//#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"


namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class MultibodyTree;
template<typename T> class RigidBodyFrame;

template <typename T>
class Body : public MultibodyTreeMember<Body<T>, BodyIndex> {
 public:
  /// Creates a new body with the provided mass properties. This body must
  /// immediately be added to a MultibodyTree.
  Body(const MassProperties<double>& mass_properties);

  RigidBodyFrame<T>& RigidlyAttachFrame(const Isometry3<T>& X_BF);

  virtual int get_num_positions() const { return 0; }

  virtual int get_num_velocities() const { return 0; }

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

 private:
  //MultibodyTree<T>* parent_tree_{nullptr};
  //BodyIndex id_;
  MassProperties<double> default_mass_properties_;

  // List of frames rigidly attached to this body.
  std::vector<FrameIndex> frames_;

  //friend class PrivateAccessAttorney;  // Give trusted member class private access.
  // Sets the parent tree of this body.
  //void set_parent_tree(MultibodyTree<T>* parent);
  //void set_id(BodyIndex id);
};

#if 0
template <typename T>
class Body<T>::PrivateAccessAttorney {
  // Only these methods can change Body<T>'s topology.
  friend Body<T>* MultibodyTree<T>::AddBody(std::unique_ptr<Body<T>> body);
  //friend void MultibodyTree<T>::Compile();

  // These methods allow the above MultibodyTree<T> methods to modify the
  // topology of a body.
  inline static void set_id(Body<T>* b, BodyIndex id) { b->set_id(id); }
  inline static void set_parent_tree(Body<T>* b, MultibodyTree<T>* p) {
    b->set_parent_tree(p);
  }
  //inline static void set_topology(Body<T>* b, const BodyTopology& t) {
  //  b->set_topology(t);
 // }
  //inline static void set_topology(Body<T>* b, const BodyTopology& t) {
  //   b->set_topology(t);
  //}
};
#endif

}  // namespace multibody
}  // namespace drake
