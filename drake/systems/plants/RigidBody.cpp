
#include "RigidBody.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include <stdexcept>

using namespace std;
using namespace Eigen;

RigidBody::RigidBody()
    : parent(nullptr),
      collision_filter_group(DrakeCollision::DEFAULT_GROUP),
      collision_filter_ignores(DrakeCollision::NONE_MASK) {
  robotnum = 0;
  position_num_start = 0;
  velocity_num_start = 0;
  body_index = 0;
  mass = 0.0;
  com = Vector3d::Zero();
  I << Matrix<double, TWIST_SIZE, TWIST_SIZE>::Zero();
}

void RigidBody::setJoint(std::unique_ptr<DrakeJoint> new_joint) {
  this->joint = move(new_joint);
}

const DrakeJoint& RigidBody::getJoint() const {
  if (joint) {
    return (*joint);
  } else {
    throw runtime_error("Joint is not initialized");
  }
}

bool RigidBody::hasParent() const { return parent != nullptr; }

void RigidBody::addVisualElement(const DrakeShapes::VisualElement& element) {
  visual_elements.push_back(element);
}

const DrakeShapes::VectorOfVisualElements& RigidBody::getVisualElements()
    const {
  return visual_elements;
}

void RigidBody::setCollisionFilter(const DrakeCollision::bitmask& group,
                                   const DrakeCollision::bitmask& ignores) {
  setCollisionFilterGroup(group);
  setCollisionFilterIgnores(ignores);
}

bool RigidBody::appendCollisionElementIdsFromThisBody(
    const string& group_name, vector<DrakeCollision::ElementId>& ids) const {
  auto group_ids_iter = collision_element_groups.find(group_name);
  if (group_ids_iter != collision_element_groups.end()) {
    ids.reserve(ids.size() + distance(group_ids_iter->second.begin(),
                                      group_ids_iter->second.end()));
    ids.insert(ids.end(), group_ids_iter->second.begin(),
               group_ids_iter->second.end());
    return true;
  } else {
    return false;
  }
}

bool RigidBody::appendCollisionElementIdsFromThisBody(
    vector<DrakeCollision::ElementId>& ids) const {
  ids.reserve(ids.size() + collision_element_ids.size());
  ids.insert(ids.end(), collision_element_ids.begin(),
             collision_element_ids.end());
  return true;
}

void RigidBody::ApplyTransformToJointFrame(
    const Eigen::Isometry3d& transform_body_to_joint) {
  I = transformSpatialInertia(transform_body_to_joint, I);
  for (auto& v : visual_elements) {
    v.SetLocalTransform(transform_body_to_joint * v.getLocalTransform());
  }
}

RigidBody::CollisionElement::CollisionElement(const CollisionElement& other)
    : DrakeCollision::Element(other), body(other.getBody()) {}

RigidBody::CollisionElement::CollisionElement(
    const Isometry3d& T_element_to_link, std::shared_ptr<RigidBody> body)
    : DrakeCollision::Element(T_element_to_link), body(body) {}

RigidBody::CollisionElement::CollisionElement(
    const DrakeShapes::Geometry& geometry, const Isometry3d& T_element_to_link,
    std::shared_ptr<RigidBody> body)
    : DrakeCollision::Element(geometry, T_element_to_link), body(body) {}

RigidBody::CollisionElement* RigidBody::CollisionElement::clone() const {
  return new CollisionElement(*this);
}

const std::shared_ptr<RigidBody>& RigidBody::CollisionElement::getBody() const {
  return this->body;
}

bool RigidBody::CollisionElement::CollidesWith(
    const DrakeCollision::Element* other) const {
  auto other_rb = dynamic_cast<const RigidBody::CollisionElement*>(other);
  bool collides = true;
  if (other_rb != nullptr) {
    collides = this->body->CollidesWith(*other_rb->body);
  }
  return collides;
}

RigidBody::CollisionElement &RigidBody::CollisionElement::add_to_collision_group(
    string &group_name) {

  // Add group_name to the list of collision elements where the CollisionElement
  // belongs.
  DrakeCollision::Element::add_to_collision_group(group_name);

  // This RigidBody::CollisionElement may have not been assigned to a body
  // yet and therefore a check is needed.
  // If the CollisionElement does not belong to a body then the collection of
  // groups where it belongs will be added to the RigidBody when calling the
  // method RigidBody::add_collision_element(const CollisionElement&).
  if(body)
    body->collision_element_groups[group_name].push_back(getId());
  return *this;
}

ostream& operator<<(ostream& out, const RigidBody& b) {
  std::string parent_joint_name =
      b.hasParent() ? b.getJoint().getName() : "no parent joint";

  std::stringstream collision_element_str;
  collision_element_str << "[";
  for (size_t ii = 0; ii < b.collision_element_ids.size(); ii++) {
    collision_element_str << b.collision_element_ids[ii];
    if (ii < b.collision_element_ids.size() - 1) collision_element_str << ", ";
  }
  collision_element_str << "]";

  out << "RigidBody\n"
      << "  - link name: " << b.linkname << "\n"
      << "  - parent joint: " << parent_joint_name << "\n"
      << "  - Collision elements IDs: " << collision_element_str.str();

  return out;
}

void RigidBody::update_collision_groups(const RigidBody::CollisionElement& collision_element) {
  for(auto it=collision_element.collision_groups_begin(); it!=collision_element.collision_groups_end();++it){
    collision_element_groups[*it].push_back(collision_element.getId());
  }
}

void RigidBody::add_collision_element(RigidBody::CollisionElement& collision_element) {
  auto id = get_RBT()->add_collision_element(collision_element);
  if (id != 0) {
    collision_element_ids.push_back(id);
    collision_element.body = this; // this is NOT a share_ptr<RigidBody> and therefore this does NOT compile
    update_collision_groups(collision_element);
  }
}
