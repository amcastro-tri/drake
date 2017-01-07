#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/mass_properties.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"


namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <typename T>
class Body {
 public:
  // Attorney-Client Idiom to allow MultibodyTree<T> to modify private topology
  // information in Body<T>.
  // see: http://stackoverflow.com/questions/3217390/clean-c-granular-friend-equivalent-answer-attorney-client-idiom/
  // This class needs to be public so that its friends methods can access it
  // from within Body. However, even if public, its only useful to its friend
  // methods and therefore it's safe to have it here.
  class TopologyAccess;

  // Option 1 for creation: create a unique_ptr to a body with a given
  // constructor, add it to a MultibodyTree which then needs to set this body's
  // parent using a *public* set_parent() (dangerous).
  /// Creates a new body with the provided mass properties. This body must
  /// immediately be added to a MultibodyTree.
  Body(const MassProperties<double>& mass_properties);

  // Option 2 for creation: A factory. Body constructors are private however
  // still accessible from within a factory. This allows setting the parent
  // without exposing a dangerous public "set_parent()" method.
  /// Creates a new body with the provided mass properties and adds it to the
  /// @p parent_tree.
  static Body<T>* CreateBody(
      MultibodyTree<T>* parent_tree,
      const MassProperties<double>& mass_properties);

  const MassProperties<double>& get_default_mass_properties() const {
    return default_mass_properties_;
  }

  const Body<T>& get_inboard_body() const;

  BodyIndex get_id() const;

  const BodyTopology& get_topology() const { return topology_; };

 private:
  MassProperties<double> default_mass_properties_;

  // Topology.
  MultibodyTree<T>* parent_tree_{nullptr};
  BodyTopology topology_;
  BodyIndex id_;

  friend class TopologyAccess;  // Give trusted member class private access.
  // Sets the parent tree of this body.
  void set_parent_tree(MultibodyTree<T>* parent);
  void set_topology(const BodyTopology& t) { topology_ = t; };
  void set_id(BodyIndex id) { topology_.id = id; }
};

template <typename T>
class Body<T>::TopologyAccess {
  // Only these methods can change Body<T>'s topology.
  friend Body<T>* MultibodyTree<T>::AddBody(std::unique_ptr<Body<T>> body);
  friend void MultibodyTree<T>::Compile();

  // These methods allow the above MultibodyTree<T> methods to modify the
  // topology of a body.
  inline static void set_id(Body<T>* b, BodyIndex id) { b->set_id(id); }
  inline static void set_parent_tree(Body<T>* b, MultibodyTree<T>* p) {
    b->set_parent_tree(p);
  }
  inline static void set_topology(Body<T>* b, const BodyTopology& t) {
    b->set_topology(t);
  }
  //inline static void set_topology(Body<T>* b, const BodyTopology& t) {
  //   b->set_topology(t);
  //}
};

}  // namespace multibody
}  // namespace drake
