#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/mass_properties.h"
#include "drake/multibody/multibody_tree/body.h"

namespace drake {
namespace multibody {

template <class V>
class RigidBody : public Body<V> {
 public:
  typedef V Vector;
  typedef typename V::Spin Spin;
  typedef typename V::SpatialVector SpatialVector;

  // Option 1 for creation: create a unique_ptr to a body with a given
  // constructor, add it to a MultibodyTree which then needs to set this body's
  // parent using a *public* set_parent() (dangerous).
  /// Creates a new body with the provided mass properties. This body must
  /// immediately be added to a MultibodyTree.
  RigidBody(const MassProperties<double>& mass_properties);

  const MassProperties<double>& get_default_mass_properties() const {
    return default_mass_properties_;
  }

  /// Computes the rigid body inertia matrix for a given, fixed, value of the
  /// flexible generalized coordinates @p qf. Since this is a rigid body, the
  /// flexible coordinates vector @p qf will have zero size and is therefore
  /// ignored.
  SpatialMatrix DoCalcSpatialInertia(const VectorX<T>& qf) const final {
    DRAKE_ASSERT(qf.size() == 0);
    return default_mass_properties_.get_mass_matrix();
  }

 private:
  MassProperties<double> default_mass_properties_;
};

}  // namespace multibody
}  // namespace drake
