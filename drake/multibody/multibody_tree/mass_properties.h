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
  /// Creates a principal rotational inertia with identical diagonal elements
  /// equal to @p I and zero products of inertia.
  /// As examples, consider the moments of inertia taken about their geometric
  /// center for a sphere or a cube.
  /// @see RotationalInertia::SolidSphere() and RotationalInertia::cube().
  RotationalInertia(const T& I) {
    I_Fo_F_ = Matrix3<T>::Zero();
    I_Fo_F_.diagonal().setConstant(I);
  }

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

  /// Computes the rotational inertia for a unit-mass solid sphere of radius
  /// @p r taken about its center.
  static RotationalInertia SolidSphere(const T& r) {
    return RotationalInertia(T(0.4) * r * r);
  }

  /// Computes the rotational inertia for a unit-mass hollow sphere consisting
  /// of an infinitesimally thin shell of radius @p r. The rotational inertia is
  /// taken about the center of the sphere.
  static RotationalInertia HollowSphere(const T& r) {
    return RotationalInertia(T(2)/T(3) * r * r);
  }

  /// Computes the rotational inertia for a unit-mass solid box taken about its
  /// geometric center. If one length is zero the inertia corresponds to that of
  /// a thin rectangular sheet. If two lengths are zero the inertia corresponds
  /// to that of a thin rod in the remaining direction.
  /// @param[in] Lx The length of the box edge in the principal x-axis.
  /// @param[in] Ly The length of the box edge in the principal y-axis.
  /// @param[in] Lz The length of the box edge in the principal z-axis.
  static RotationalInertia SolidBox(const T& Lx, const T& Ly, const T& Lz) {
    const T one_twelfth = T(1) / T(12);
    const T Lx2 = Lx * Lx, Ly2 = Ly * Ly, Lz2 = Lz * Lz;
    return RotationalInertia(
        one_twelfth * (Ly2 + Lz2),
        one_twelfth * (Lx2 + Lz2),
        one_twelfth * (Lx2 + Ly2));
  }

  /// Computes the rotational inertia for a unit-mass solid cube (a box with
  /// equal sized sides) taken about its geometric center.
  /// @param[in] L The length of each of the cube's sides.
  static RotationalInertia SolidCube(const T& L) {
    return SolidBox(L, L, L);
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

  /// Returns the MassProperties object for a body of infinite mass.
  /// Center of mass and rotational inertia are meaningless.
  static MassProperties<double> InfiniteMass() {
    return MassProperties<double>(
        Eigen::NumTraits<double>::infinity(),
        Eigen::Vector3d::Zero(), RotationalInertia<double>(0.0));
  }

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
