#pragma once

#include <memory>
#include <sstream>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

template <typename T>
class RotationalInertia {
 public:
  enum {
    // By default RotationalInertia only works on the upper part of the
    // underlying Eigen matrix.
    TriangularViewInUse = Eigen::Upper,
    // The strictly lower part is set to NaN to quickly detect when used by
    // error.
    TriangularViewNotInUse = Eigen::StrictlyLower
  };

  /// Creates a principal rotational inertia with identical diagonal elements
  /// equal to @p I and zero products of inertia.
  /// As examples, consider the moments of inertia taken about their geometric
  /// center for a sphere or a cube.
  /// @see RotationalInertia::SolidSphere() and RotationalInertia::cube().
  RotationalInertia(const T& I) {
    SetZero();
    I_Bo_F_.diagonal().setConstant(I);
  }

  /// Create a principal axes rotational inertia matrix for wich off-diagonal
  /// elements are zero.
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz) {
    SetZero();
    I_Bo_F_.diagonal() = Vector3<T>(Ixx, Iyy, Izz);
  }

  /// Creates a general rotational inertia matrix with non-zero off-diagonal
  /// elements.
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz,
                    const T& Ixy, const T& Ixz, const T& Iyz) {
    // The TriangularViewNotInUse is left initialized to NaN.
    auto triangular = I_Bo_F_.template selfadjointView<TriangularViewInUse>();
    triangular(0, 0) = Ixx; triangular(1, 1) = Iyy; triangular(2, 2) = Izz;
    triangular(0, 1) = Ixy; triangular(0, 2) = Ixz; triangular(1, 2) = Iyz;
  }

  int rows() const { return 3;}

  int cols() const { return 3;}

  Vector3<T> get_moments() const { return I_Bo_F_.diagonal(); }

  Vector3<T> get_products() const {
    return Vector3<T>(
        I_Bo_F_(0,1), I_Bo_F_(0,2), I_Bo_F_(1,2));
  }

  T& operator()(int i, int j) {
    // Overwrites local copies of i and j.
    check_and_swap(&i, &j);
    return I_Bo_F_(i, j);
  }

  const T& operator()(int i, int j) const {
    // Overwrites local copies of i and j.
    check_and_swap(&i, &j);
    return I_Bo_F_(i, j);
  }

  /// Returns a view to the symmetric part of the matrix in use by
  /// RotationalInertia.
  const Eigen::SelfAdjointView<const Matrix3<T>, TriangularViewInUse>
  get_symmetric_matrix_view() const {
    return I_Bo_F_.template selfadjointView<TriangularViewInUse>();
  }

  /// Returns a constant reference to the underlying Eigen matrix. Notice that
  /// since RotationalInertia only uses the
  /// RotationalInertia::TriangularViewInUse portion of this
  /// matrix, the RotationalInertia::TriangularViewNotInUse part will be set to
  /// have NaN entries.
  const Matrix3<T>& get_matrix() const { return I_Bo_F_; }

  /// Get a copy to a full Matrix3 representation for this rotational inertia
  /// including both lower and upper triangular parts.
  Matrix3<T> CopyToFullMatrix3() const { return get_symmetric_matrix_view(); }

  void SetToNaN() {
    I_Bo_F_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  void SetZero() {
    //I_Bo_F_.setConstant(std::numeric_limits<
    //    typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
    // RotationalInertia only works with the upper-triangular portion of the
    // underlying Eigen matrix. The lower part is set to NaN to quickly detect
    // when the lower part is mistakenly used.
    I_Bo_F_.template triangularView<TriangularViewInUse>() = Matrix3<T>::Zero();
  }

  /// Assignment operator from a general Eigen expression.
  // This method allows you to assign Eigen expressions to a RotationalInertia.
  template<typename Derived>
  RotationalInertia& operator=(const Eigen::MatrixBase<Derived>& EigenMatrix)
  {
    // Static asserts that EigenMatrix is of fixed size 3x3.
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
    I_Bo_F_ = EigenMatrix;
    return *this;
  }

  RotationalInertia ReExpressIn(const Matrix3<T>& R_AF) {
    RotationalInertia<T> I_Bo_A;
    I_Bo_A = R_AF *
        I_Bo_F_.template selfadjointView<TriangularViewInUse>() *
        R_AF.transpose();
    return I_Bo_A;
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

  /// Computes the rotational inertia for a unit-mass rod along the z-axis
  /// rotationg about its center.
  /// @param[in] r The radius of the rod.
  /// @param[in] L The length of the rod.
  static RotationalInertia SolidRod(const T& r, const T& L) {
    const T Iz = r * r / T(2);
    const T Ix = L * L / T(12);
    return RotationalInertia(Ix, Ix, Iz);
  }

  /// Computes the rotational inertia for a unit-mass rod along the z-axis
  /// rotationg about one end.
  /// @param[in] r The radius of the rod.
  /// @param[in] L The length of the rod.
  static RotationalInertia SolidRodAboutEnd(const T& r, const T& L) {
    const T Iz = r * r / T(2);
    const T Ix = L * L / T(3);
    return RotationalInertia(Ix, Ix, Iz);
  }

 private:
  RotationalInertia() {}

  static void check_and_swap(int* i, int* j) {
    const bool swap =
        (int(TriangularViewInUse) == int(Eigen::Upper) && *i > *j) ||
        (int(TriangularViewInUse) == int(Eigen::Lower) && *i < *j);
    if (swap) std::swap(*i , *j);
  }
  // Inertia matrix about frame B's origin Bo expressed in frame F.
  // Frame F and origin Bo are implicit here, RotationalInertia only keeps track
  // of the inertia measures in this frame F. Users are responsible for keeping 
  // track of the frame in which a particular inertia is expressed in.
  // Initially set to NaN to aid finding when by mistake we use the strictly
  // lower part of the matrix. Only the upper part should be used.
  Matrix3<T> I_Bo_F_{Matrix3<T>::Constant(std::numeric_limits<
      typename Eigen::NumTraits<T>::Literal>::quiet_NaN())};
};

template <typename T> inline
std::ostream& operator<<(std::ostream& o,
                         const RotationalInertia<T>& I) {
  // This allow us to set the number of decimal places to print.
  // 0 uses default precision.
  const std::streamsize precision = 0;

  int width = 0;
  std::streamsize old_precision = 0;
  std::ios_base::fmtflags old_flags = o.flags();
  if(precision) {
    old_precision = o.precision(precision);
    o << std::fixed;
  }

  // Computes largest width so that we can align columns for a prettier format.
  // Idea taken from: Eigen::internal::print_matrix() in Eigen/src/Core/IO.h
  for(int j = 0; j < I.cols(); ++j) {
    for (int i = 0; i < I.rows(); ++i) {
      std::stringstream sstr;
      sstr.copyfmt(o);
      sstr << I(i, j);
      width = std::max<int>(width, int(sstr.str().length()));
    }
  }

  // Outputs to stream.
  for(int i = 0; i < I.rows(); ++i) {
    o << "[";
    if (width) o.width(width);
    o << I(i, 0);
    for (int j = 1; j < I.cols(); ++j) {
      o << ", ";
      if (width) o.width(width);
      o << I(i, j);
    }
    o << "]" << std::endl;
  }
  if(precision) {
    o.precision(old_precision);
    o.flags(old_flags);
  }
  return o;
}

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
