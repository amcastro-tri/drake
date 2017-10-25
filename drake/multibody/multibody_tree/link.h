#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/spatial_inertia.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <typename T>
class LinkFrame final : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinkFrame)

  const Link<T>& get_link() const {
    DRAKE_DEMAND(this->get_link_pointer() != nullptr);
    return *this->get_link_pointer();
  }

  /// @name Advanced level methods.
  /// @{
  /// Returns the pose `X_BF` of `this` frame F in the body frame B associated
  /// with this frame.
  /// In particular, if `this` **is** the body frame B, this method directly
  /// returns the identity transformation.
  Isometry3<T> CalcPoseInBodyFrame(
      const systems::Context<T>& context) const override {
    DRAKE_DEMAND(implementation_ != nullptr);
    return implementation_->CalcPoseInBodyFrame(context);
  }

  /// Given the offset pose `X_FQ` of a frame Q in `this` frame F, this method
  /// computes the pose `X_BQ` of frame Q in the body frame B to which this
  /// frame is attached.
  /// In other words, if the pose of `this` frame F in the body frame B is
  /// `X_BF`, this method computes the pose `X_BQ` of frame Q in the body frame
  /// B as `X_BQ = X_BF * X_FQ`.
  /// In particular, if `this` **is**` the body frame B, i.e. `X_BF` is the
  /// identity transformation, this method directly returns `X_FQ`.
  /// Specific frame subclasses can override this method to provide faster
  /// implementations if needed.
  Isometry3<T> CalcOffsetPoseInBody(
      const systems::Context<T>& context,
      const Isometry3<T>& X_FQ) const override {
    DRAKE_DEMAND(implementation_ != nullptr);
    return implementation_->CalcOffsetPoseInBody(context, X_FQ);
  }
  /// @}

  // Hide the following section for internal methods from Doxygen.
  // These methods are intended for internal use only.
#ifndef DRAKE_DOXYGEN_CXX
  // (Internal) Get the underlying implementation (a BodyFrame) for this frame.
  const BodyFrame<T>& get_implementation() const {
    DRAKE_DEMAND(implementation_ != nullptr);
    return *implementation_;
  }
#endif

 protected:
  // Frame<T>::DoCloneToScalar() overrides.
  std::unique_ptr<Frame<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Frame<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  // Link<T> and LinkFrame<T> are natural allies. A LinkFrame object is created
  // every time a Link object is created and they are associated with each
  // other.
  friend class Link<T>;

  // Make LinkFrame templated on any other scalar type a friend of
  // LinkFrame<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private methods from BodyFrame<T>.
  template <typename> friend class LinkFrame;

  // Only Link objects can create BodyFrame objects since Body is a friend of
  // BodyFrame.
  explicit LinkFrame(const Link<T>& link) : Frame<T>(&link, nullptr) {}

  // Helper method to make a clone templated on any other scalar type.
  // This method holds the common implementation for the different overrides to
  // DoCloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<Frame<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // The internal body frame this link frame represents.
  const BodyFrame<T>* implementation_{nullptr};
};

namespace internal {
// This is a class used by MultibodyTree internals to create the implementation
// for a particular joint object.
template <typename T>
class LinkImplementationBuilder;
}  // namespace internal

/// A %Link models the kinematical relationship which characterizes the
/// possible relative motion between two bodies.
/// The two bodies connected by a %Link object are referred to as the
/// _parent_ and _child_ bodies. Although the terms _parent_ and _child_ are
/// sometimes used synonymously to describe the relationship between inboard and
/// outboard bodies in multibody models, this usage is wholly unrelated and
/// implies nothing about the inboard-outboard relationship between the bodies.
/// A %Link is a model of a physical kinematic constraint between two bodies,
/// a constraint that in the real physical system does not even allude to the
/// ordering of the bodies.
///
/// In Drake we define a frame F rigidly attached to the parent body P with pose
/// `X_PF` and a frame M rigidly attached to the child body B with pose `X_BM`.
/// A %Link object specifies a kinematic relation between frames F and M,
/// which in turn imposes a kinematic relation between bodies P and B.
///
/// Typical joints include the ball joint, to allow unrestricted rotations about
/// a given point, the revolute joint, that constraints two bodies to rotate
/// about a given common axis, etc.
///
/// Consider the following example to build a simple pendulum system:
///
/// @code
/// MultibodyTree<double> model;
/// // ... Code here to setup quantities below as mass, com, etc. ...
/// const Body<double>& pendulum =
///   model.AddBody<RigidBody>(SpatialInertia<double>(mass, com, unit_inertia));
/// // We will connect the pendulum body to the world using a RevoluteLink.
/// // In this simple case the parent body P is the model's world body and frame
/// // F IS the world frame.
/// // Additionally, we need to specify the pose of frame M on the pendulum's
/// // body frame B.
/// // Say we declared and initialized X_BM...
/// const RevoluteLink<double>& elbow =
///   model.AddLink<RevoluteLink>(
///     "Elbow",                /* joint name */
///     model.get_world_body(), /* parent body */
///     {},                     /* frame F IS the world frame W */
///     pendulum,               /* child body, the pendulum */
///     X_BM,                   /* pose of frame M in the body frame B */
///     Vector3d::UnitZ());     /* revolute axis in this case */
/// @endcode
///
/// @warning Do not ever attempt to instantiate and manipulate %Link objects
/// on the stack; it will fail. Add joints to your model using the provided API
/// MultibodyTree::AddLink() as in the example above.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Link : public MultibodyTreeElement<Link<T>, LinkIndex>  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Link)

  /// Constructs a %RigidBody with the given default SpatialInertia.
  /// @param[in] M_BBo_B
  ///   Spatial inertia of `this` body B about the frame's origin `Bo` and
  ///   expressed in the body frame B.
  /// @note See @ref multibody_spatial_inertia for details on the monogram
  /// notation used for spatial inertia quantities.
  Link(const std::string& name, const SpatialInertia<double> M_BBo_B);

  /// Returns the name of this joint.
  const std::string& get_name() const { return name_; }

  /// Returns the default value of this link's spatial inertia about Bo (body B's
  /// origin), expressed in B (this body's frame). This value is initially
  /// supplied at construction when specifying this body's SpatialInertia.
  /// @retval G_BBo_B rigid body B's unit inertia about Bo, expressed in B.
  const SpatialInertia<double>& get_default_spatial_inertia() const {
    return default_spatial_inertia_;
  }

  /// Returns a const reference to the associated BodyFrame.
  const LinkFrame<T>& get_link_frame() const {
    return link_frame_;
  }

  // Hide the following section from Doxygen.
#ifndef DRAKE_DOXYGEN_CXX

  // (Internal) Method used by MultibodyTree when adding links to access a
  // mutable reference to this link's frame.
  LinkFrame<T>& get_mutable_link_frame() {
    return link_frame_;
  }

  // NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  // be created. This method is intended to be called by
  // MultibodyTree::CloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<Link<ToScalar>> CloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const {
    std::unique_ptr<Link<ToScalar>> joint_clone = DoCloneToScalar(tree_clone);

    std::unique_ptr<typename Link<ToScalar>::LinkImplementation>
        implementation_clone =
        this->get_implementation().template CloneToScalar<ToScalar>(tree_clone);
    joint_clone->OwnImplementation(std::move(implementation_clone));

    return std::move(joint_clone);
  }
#endif
  // End of hidden Doxygen section.

 protected:
  /// (Advanced) Structure containing all the information needed to build the
  /// MultibodyTree implementation for a %Link. At MultibodyTree::Finalize() a
  /// %Link creates a BluePrint of its implementation with MakeModelBlueprint()
  /// so that MultibodyTree can build an implementation for it.
  struct BluePrint {
    std::vector<std::unique_ptr<RigidBody<T>>> bodies_;
    // TODO(amcastro-tri): constraints, for instance, to weld two ghost bodies.
  };

  /// (Advanced) A Link is implemented in terms of MultibodyTree elements such
  /// as bodies, mobilizers, force elements and constraints. This object
  /// contains the internal details of the MultibodyTree implementation for a
  /// joint. The implementation does not own the MBT elements, it just keeps
  /// references to them.
  /// This is intentionally made a protected member so that derived classes have
  /// access to its definition.
  struct LinkImplementation {
    /// Default constructor to create an empty implementation. Used by
    /// Link::CloneToScalar().
    LinkImplementation() {}

    /// This constructor creates an implementation for `this` link from the
    /// blueprint provided.
    explicit LinkImplementation(const BluePrint& blue_print) {
      DRAKE_DEMAND(static_cast<int>(blue_print.bodies_.size()) != 0);
      for (const auto& body : blue_print.bodies_) {
        bodies_.push_back(body.get());
      }
    }

    /// Returns the number of bodies in this implementation.
    int get_num_bodies() const {
      return static_cast<int>(bodies_.size());
    }

    // Hide the following section from Doxygen.
#ifndef DRAKE_DOXYGEN_CXX
    // Helper method to be called within Link::CloneToScalar() to clone its
    // implementation to the appropriate scalar type.
    template <typename ToScalar>
    std::unique_ptr<typename Link<ToScalar>::LinkImplementation>
    CloneToScalar(const MultibodyTree<ToScalar>& tree_clone) const {
      auto implementation_clone =
          std::make_unique<typename Link<ToScalar>::LinkImplementation>();
      for (const Mobilizer<T>* body : bodies_) {
        const Mobilizer<ToScalar>* body_clone =
            &tree_clone.get_variant(*body);
        implementation_clone->bodies_.push_back(body_clone);
      }
      return std::move(implementation_clone);
    }
#endif
    // End of hidden Doxygen section.

    /// References (raw pointers) to the bodies that make part of this
    /// implementation.
    std::vector<const RigidBody<T>*> bodies_;
    // TODO(amcastro-tri): constraints, for instance, to weld two ghost bodies.
  };

  // Implements MultibodyTreeElement::DoSetTopology(). Links have no topology
  // though we could require them to have one in the future.
  void DoSetTopology(const MultibodyTreeTopology&) {}

  /// @name Methods to make a clone templated on different scalar types.
  /// @{
  /// Clones this %Link (templated on T) to a joint templated on `double`.
  std::unique_ptr<Link<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const;

  /// Clones this %Link (templated on T) to a joint templated on AutoDiffXd.
  std::unique_ptr<Link<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const;
  /// @}

  /// This method must be implemented by derived classes in order to provide
  /// LinkImplementationBuilder a BluePrint of their internal implementation
  /// LinkImplementation.
  // TODO(amcastro-tri): Add arguments to indicate whether to split into ghost
  // bodies or not.
  std::unique_ptr<BluePrint> MakeImplementationBlueprint() const {
    auto blue_print = std::make_unique<BluePrint>();
    auto body = std::make_unique<RigidBody<T>>(default_spatial_inertia_);
    body->set_link(this);
    blue_print->bodies_.push_back(std::move(body));
    return std::move(blue_print);
  }

  /// Returns a const reference to the internal implementation of `this` joint.
  /// @warning The MultibodyTree model must have already been finalized, or
  /// this method will abort.
  const LinkImplementation& get_implementation() const {
    // The MultibodyTree must have been finalized for the implementation to be
    // valid.
    DRAKE_DEMAND(this->get_parent_tree().topology_is_valid());
    return *implementation_;
  }

 private:
  // Make all other Link<U> objects a friend of Link<T> so they can make
  // Link<ToScalar>::LinkImplementation from CloneToScalar<ToScalar>().
  template <typename> friend class Link;

  // LinkImplementationBuilder is a friend so that it can access the
  // Link<T>::BluePrint and protected method MakeImplementationBlueprint().
  friend class internal::LinkImplementationBuilder<T>;

  // When an implementation is created, either by
  // internal::LinkImplementationBuilder or by Link::CloneToScalar(), this
  // method is called to pass ownership of an implementation to the Link.
  void OwnImplementation(std::unique_ptr<LinkImplementation> implementation) {
    implementation_ = std::move(implementation);
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Link<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  std::string name_;

  SpatialInertia<double> default_spatial_inertia_;

  LinkFrame<T> link_frame_;

  // The Link<T> implementation:
  std::unique_ptr<LinkImplementation> implementation_;
};

}  // namespace multibody
}  // namespace drake
