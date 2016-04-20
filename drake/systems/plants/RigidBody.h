#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/joints/DrakeJoint.h"

class RigidBodyTree; // forward declaration

class DRAKERBM_EXPORT RigidBody {
 private:
  std::unique_ptr<DrakeJoint> joint;
  DrakeCollision::bitmask collision_filter_group;
  DrakeCollision::bitmask collision_filter_ignores;
  std::shared_ptr<RigidBodyTree> rbt_parent_; /*!< The RigidBodyTree where this body belongs */

 public:
  RigidBody();

  void setJoint(std::unique_ptr<DrakeJoint> joint);
  const DrakeJoint& getJoint() const;

  bool hasParent() const;

  /**
   * \brief Returns the RigidBodyTree where this RigidBody belongs
   * (const version).
   *
   * The parent RigidBodyTree is well defined since each RigidBody can only
   * belong to one RigidBodyTree.
   *
   * \return The parent RigidBodyTree.
   *
   * \throw A std::runtime_error if the RigidBody does not have a parent
   * RigidBodyTree
   */
  std::shared_ptr<RigidBodyTree> get_RBT() const {
    if(rbt_parent_) return rbt_parent_;
    throw std::runtime_error("Bug?: RigidBody \""+linkname+
        "\" does not belong to a RigidBodyTree.");
  }

  /**
   * \brief Returns the RigidBodyTree where this RigidBody belongs
   * (non-const version).
   *
   * The parent RigidBodyTree is well defined since each RigidBody can only
   * belong to one RigidBodyTree.
   *
   * \return The parent RigidBodyTree.
   *
   * \throw A std::runtime_error if the RigidBody does not have a parent
   * RigidBodyTree
   */
  std::shared_ptr<RigidBodyTree> get_RBT() {
    if(rbt_parent_) return rbt_parent_;
    throw std::runtime_error("Bug?: RigidBody \""+linkname+
        "\" does not belong to a RigidBodyTree.");
  }

  /**
   * \brief Add a visual element to this rigid body.
   *
   * Each RigidBody can have more than one visual element and be visualized as
   * the union of several DrakeShapes::VisualElement's. Therefore this
   * method can be called multiple times on a single RigidBody.
   *
   * \param visual_element The CollisionElement being added to this RigidBody.
   *
   * \see addCollisionElement.
   *
   * TODO(amcastro-tri): rename to follow Google's style guide.
   */
  void addVisualElement(const DrakeShapes::VisualElement& visual_element);


  const DrakeShapes::VectorOfVisualElements& getVisualElements() const;

  void setCollisionFilter(const DrakeCollision::bitmask& group,
                          const DrakeCollision::bitmask& ignores);

  const DrakeCollision::bitmask& getCollisionFilterGroup() const {
    return collision_filter_group;
  }
  void setCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_group = group;
  }

  const DrakeCollision::bitmask& getCollisionFilterIgnores() const {
    return collision_filter_ignores;
  }
  void setCollisionFilterIgnores(const DrakeCollision::bitmask& ignores) {
    this->collision_filter_ignores = ignores;
  }

  void addToCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_group |= group;
  }
  void ignoreCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_ignores |= group;
  }
  void collideWithCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_ignores &= ~group;
  }

  bool adjacentTo(const RigidBody& other) const {
    return ((parent.get() == &other && !(joint && joint->isFloating())) ||
            (other.parent.get() == this &&
             !(other.joint && other.joint->isFloating())));
  }

  bool CollidesWith(const RigidBody& other) const {
    bool ignored =
        this == &other || adjacentTo(other) ||
        (collision_filter_group & other.getCollisionFilterIgnores()).any() ||
        (other.getCollisionFilterGroup() & collision_filter_ignores).any();
    return !ignored;
  }

  bool appendCollisionElementIdsFromThisBody(
      const std::string& group_name,
      std::vector<DrakeCollision::ElementId>& ids) const;

  bool appendCollisionElementIdsFromThisBody(
      std::vector<DrakeCollision::ElementId>& ids) const;

  /**
   * Transforms all of the visual, collision, and inertial elements associated
   * with this body to the proper joint frame.  This is necessary, for instance,
   * to support SDF loading where the child frame can be specified independently
   * from the joint frame. In our RigidBodyTree classes, the body frame IS the
   * joint frame.
   *
   * @param transform_body_to_joint The transform from this body's frame to the
   * joint's frame.
   */
  void ApplyTransformToJointFrame(
      const Eigen::Isometry3d& transform_body_to_joint);

 public:
  std::string linkname;
  std::string model_name;  // todo: replace robotnum w/ model_name
  int robotnum;            // uses 0-index. starts from 0
  // note: it's very ugly, but parent, dofnum, and pitch also exist currently
  // (independently) at the RigidBodyTree level to represent the featherstone
  // structure.  this version is for the kinematics.
  std::shared_ptr<RigidBody> parent;
  int body_index;
  int position_num_start;
  int velocity_num_start;

  DrakeShapes::VectorOfVisualElements visual_elements;

  std::vector<DrakeCollision::ElementId> collision_element_ids;
  std::map<std::string, std::vector<DrakeCollision::ElementId> >
      collision_element_groups;

  Eigen::Matrix3Xd contact_pts;

  double mass;
  Eigen::Vector3d com;
  Eigen::Matrix<double, TWIST_SIZE, TWIST_SIZE> I;

  friend std::ostream& operator<<(std::ostream& out, const RigidBody& b);

  // FIXME: move to a better place:
  class DRAKERBM_EXPORT CollisionElement : public DrakeCollision::Element {
    friend class RigidBody;
   public:
    CollisionElement(const CollisionElement& other);
    CollisionElement(const Eigen::Isometry3d& T_element_to_link,
                     std::shared_ptr<RigidBody> body);
    CollisionElement(const DrakeShapes::Geometry& geometry,
                     const Eigen::Isometry3d& T_element_to_link,
                     std::shared_ptr<RigidBody> body = std::shared_ptr<RigidBody>());
    virtual ~CollisionElement() {}

    CollisionElement* clone() const override;

    const std::shared_ptr<RigidBody>& getBody() const;

    bool CollidesWith(const DrakeCollision::Element* other) const override;

    /**
     * \brief Adds to a collision group.
     *
     * Objects within the provided collision group are not checked for
     * collisions.
     *
     * \param[in] group_name The name of the collision group.
     *
     * \return this
     */
    RigidBody::CollisionElement& add_to_collision_group(std::string& group_name);

#ifndef SWIG
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

   private:
    std::shared_ptr<RigidBody> body;
  };

  /**
   * \brief Add a collision element to this rigid body.
   *
   * Each RigidBody can have more than one CollisionElement in the same way each
   * RigidBody can also have more than one DrakeShapes::VisualElement.
   * Therefore this method can be called multiple times on a same RigidBody to
   * add multiple RigidBody::CollisionElements.
   * Notice the parallelism of the interface provided with that for
   * RigidBody::addVisualElement.
   *
   * \param collision_element The CollisionElement being added to this RigidBody.
   *
   * \see addVisualElement.
   *
   * TODO(amcastro-tri): To have an interface entirely parallel to that for
   * VisualElement's, CollisionElement should taken out from RigidBody and be
   * moved to the namespace DrakeCollision.
   *
   */
  void add_collision_element(CollisionElement& collision_element);

  void update_collision_groups(const CollisionElement &collision_element);

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
