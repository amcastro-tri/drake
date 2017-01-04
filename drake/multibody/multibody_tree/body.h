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
  /// Creates a new body with the provided mass properties and adds it to the
  /// @p parent_tree.
  static Body<T>* CreateBody(
      MultibodyTree<T>* parent_tree,
      const MassProperties<double>& mass_properties);

  const MassProperties<double>& get_default_mass_properties() const {
    return default_mass_properties_;
  }

 private:
  // Constructs a body with default mass properties @p mass_properties.
  // Bodies can only be created through Body::CreateBody().
  Body(MultibodyTree<T>* parent_tree,
       const MassProperties<double>& mass_properties);

  MultibodyTree<T>* parent_tree_{nullptr};
  MassProperties<double> default_mass_properties_;
};

}  // namespace multibody
}  // namespace drake
