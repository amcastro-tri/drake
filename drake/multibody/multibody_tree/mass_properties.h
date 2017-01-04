#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {

template <typename T>
class RotationalInertia {
 public:
  /// Create a principal axes rotational inertia matrix for wich off-diagonal
  /// elements are zero.
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz) {
    I_Fo_F_ = Matrix3<T>::Zero();
    I_Fo_F_.diagonal() = Vector3<T>(Ixx, Iyy, Izz);
  }

  /// Creates a general rotational inertia matrix with non-zero off-diagonal
  /// elements.
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz,
                    const T& Ixy, const T& Ixz, const T& Iyz);

  Vector3<T> get_moments() const { return I_Fo_F_.diagonal(); }
  Vector3<T> get_products() const {
    return Vector3<T>(
        I_Fo_F_(0,1), I_Fo_F_(0,2), I_Fo_F_(1,2));
  }

 private:
  // Inertia matrix about frame F's origin Fo expressed in frame F.
  // Frame F is implicit here, RotationalInertia only keeps track of the inertia
  // measures in this frame F. Users are responsible for keeping track of the
  // frame in which a particular inertia is expressed in.
  Matrix3<T> I_Fo_F_;
};

/// This class represents the mass properties of a body measured and expressed
/// in the body's frame B. The center of mass is a vector from the orgin of the
/// body's frame Bo. The rotational intertia is computed about Bo and expressed
/// in B. This class only stores the mass property measures in the body frame B,
/// the body frame B itself is implicit.
template <typename T>
class MassProperties {
 public:
  /// Constructor from mass, center of mass and rotational inertia.
  /// @param mass The mass of the bodyy.
  /// @param com_B Body center of mass measured and expressed in B.
  /// @param I_Bo_B Rotational inertia about B's origin Bo expressed in B.
  MassProperties(const T& mass, const Vector3<T>& com_B,
                 const RotationalInertia<T>& I_Bo_B) :
      mass_(mass), com_B_(com_B), I_Bo_B_(I_Bo_B) { }

  const T& get_mass() const { return mass_; }
  const Vector3<T>& get_com() const { return com_B_; }
  const RotationalInertia<T>& get_rotational_inertia() const { return I_Bo_B_; }
 private:
  T mass_;
  // center of mass measured and expressed in B.
  Vector3<T> com_B_;
  // Rotational inertia around Bo and expressed in B.
  RotationalInertia<T> I_Bo_B_;
};

template <typename T>
static inline std::ostream& operator<<(
    std::ostream& o, const MassProperties<T>& mp) {
  return o
      << " mass = " << mp.get_mass() << std::endl
      << " com = " << mp.get_com().transpose() << std::endl
      << " Ixx, Iyy, Izz = " <<
      mp.get_rotational_inertia().get_moments().transpose() << std::endl
      << " Ixy, Ixz, Iyz = " <<
      mp.get_rotational_inertia().get_products().transpose() << std::endl;
}


}  // namespace multibody
}  // namespace drake
