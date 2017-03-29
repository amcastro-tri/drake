#pragma once

#include <memory>
#include <sstream>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

/// This class represents the mass properties of a body measured and expressed
/// in the body's frame B. The center of mass is a vector from the orgin of the
/// body's frame Bo. The rotational intertia is computed about Bo and expressed
/// in B. This class only stores the mass property measures in the body frame B,
/// the body frame B itself is implicit.
template <typename T>
class MassProperties {
 public:
  /// Constructor from mass, center of mass and unit rotational inertia.
  /// @param mass The mass of the bodyy.
  /// @param com_B Body center of mass measured and expressed in B.
  /// @param G_Bo_B Unit rotational inertia about B's origin Bo expressed in B.
  MassProperties(const T& mass, const Vector3<T>& com_B,
                 const UnitInertia<T>& G_Bo_B) :
      mass_(mass), com_B_(com_B), G_Bo_B_(G_Bo_B) { }

  const T& get_mass() const { return mass_; }
  const Vector3<T>& get_com() const { return com_B_; }
  const UnitInertia<T>& get_unit_inertia() const { return G_Bo_B_; }

  /// Returns the MassProperties object for a body of infinite mass.
  /// Center of mass and rotational inertia are meaningless.
  static MassProperties<double> InfiniteMass() {
    return MassProperties<double>(
        Eigen::NumTraits<double>::infinity(),
        Eigen::Vector3d::Zero(), UnitInertia<double>(0.0));
  }

 private:
  T mass_;
  // center of mass measured and expressed in B.
  Vector3<T> com_B_;
  // Unit inertia around Bo and expressed in B.
  UnitInertia<T> G_Bo_B_;
};

template <typename T>
static inline std::ostream& operator<<(
    std::ostream& o, const MassProperties<T>& mp) {
  return o
      << " mass = " << mp.get_mass() << std::endl
      << " com = " << mp.get_com().transpose() << std::endl
      << " Ixx, Iyy, Izz = " <<
      mp.get_unit_inertia().get_moments().transpose() << std::endl
      << " Ixy, Ixz, Iyz = " <<
      mp.get_unit_inertia().get_products().transpose() << std::endl;
}


}  // namespace multibody
}  // namespace drake
