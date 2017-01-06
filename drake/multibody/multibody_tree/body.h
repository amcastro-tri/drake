#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/mass_properties.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <typename T>
class Body {
 public:

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

  /// Sets the parent tree of this body.
  /// This methods needs to be public so that MultibodyTree::AddBody() can
  /// access it. However it is dangerous to expose it to users.
  /// Users should never call this method.
  void set_parent_tree(MultibodyTree<T>* parent);

  const MassProperties<double>& get_default_mass_properties() const {
    return default_mass_properties_;
  }

 private:
  MultibodyTree<T>* parent_tree_{nullptr};
  MassProperties<double> default_mass_properties_;
};

}  // namespace multibody
}  // namespace drake
