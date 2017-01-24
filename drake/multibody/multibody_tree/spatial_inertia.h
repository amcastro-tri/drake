#pragma once

#include <memory>
#include <sstream>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/small_vectors.h"
#include "drake/multibody/multibody_tree/rotational_inertia.h"

#include <iostream>
#include <sstream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

template <typename T>
class SpatialInertia {
 public:
  /// Default SpatialInertia constructor initializes mass, center of mass and
  /// rotational inertia to invalide NaN's for a quick detection of
  /// un-initialized values.
  SpatialInertia() {}

  /// Constructs a spatial inertia from a given mass, center of mass and
  /// rotational inertia. The center of mass is measured from an origin Bo, i.e.
  /// the center of mass is a vector from origin `Bo` to the body's center of
  /// mass `Bc`, `com = p_BoBc`. The rotational inertia is exptected to be
  /// computed about the same origin `Bo`.
  /// Both center of mass `com` and rotational inertia `I` are expected to be
  /// expressed on a same frame `F`.
  /// @param[in] mass The mass of the body.
  /// @param[in] com The center of mass of the body measured from an origin `Bo`
  /// and expressed in a frame `F`.
  /// @param[in] I Rotational inertia of the body computed about origin `Bo` and
  /// expressed in a frame `F`.
  SpatialInertia(
      const T& mass, const Vector3<T>& com, const RotationalInertia<T>& I) :
      mass_(mass), p_BoBc_F_(com), I_Bo_F_(I) {}

  // Default copy constructor and copy assignment.
  SpatialInertia(const SpatialInertia<T>& other) = default;
  SpatialInertia& operator=(const SpatialInertia<T>& other) = default;

  const T& get_mass() const { return mass_;}

  const Vector3<T>& get_com() const { return p_BoBc_F_;}

  const RotationalInertia<T>& get_rotational_inertia() const { return I_Bo_F_;}

  /// Performs a number of checks to verify that this is a physically valid
  /// spatial inertia.
  /// The chekcs performed are:
  /// - No NaN entries.
  /// - Positive mass.
  /// - Valid rotational inertia,
  /// @see RotationalInertia::IsValidRotationalInertia().
  bool IsValidSpatialInertia() const {
    if (isnan(mass_)) return false;
    if (p_BoBc_F_.array().isNaN().any()) return false;
    if (mass_ <= T(0)) return false;
    if (!I_Bo_F_.IsValidRotationalInertia()) return false;
    return true;  // All tests passed.
  }

  /// Given this spatial inertia `M_Bo_F` about `Bo` and expressed in frame `F`,
  /// this method re-expresses the same spatial inertia in another frame `A`.
  /// This operation is performed in-place modifying the original object.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns A references to `this` object which now is the same spatial inertia
  /// about `Bo` but expressed in frame `A`.
  SpatialInertia& ReExpressInPlace(const Matrix3<T>& R_AF) {
    p_BoBc_F_ = R_AF * p_BoBc_F_;
    I_Bo_F_.ReExpressInPlace(R_AF);
    return *this;
  }

  /// Given this spatial inertia `M_Bo_F` about `Bo` and expressed in frame `F`,
  /// this method computes the same spatial inertia but re-expressed in another
  /// frame `A`.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns M_Bo_A The same spatial inertia about `Bo` but expressed in
  /// frame `A`.
  SpatialInertia ReExpress(const Matrix3<T>& R_AF) const {
    return SpatialInertia(*this).ReExpressInPlace(R_AF);
  }

  /// This methods perfomrs the "parallel axis theorem" for spatial inertias:
  /// given this spatial inertia `M_Bo_F` about `Bo` and expressed in frame `F`,
  /// this method modifies this spatial inertia to be computed about a new
  /// origin Xo. The result still is expressed in frame `F`.
  /// This operation is performed in-place modifying the original object.
  /// @see ShiftOrigin() which does not modify this object.
  ///
  /// See Section 2.1.2, p. 20 of A. Jain's book.
  ///
  /// @param[in] p_BoXo_F Vector from the original origin `Bo` to the new origin
  /// `Xo`, expressed in the spatial inertia frame `F`.
  /// @returns `M_Xo_F` This same spatial inertia but computed about
  /// origin `Xo`.
  SpatialInertia& ShiftOriginInPlace(const Vector3<T>& p_BoXo_F) {
    using math::CrossProductMatrixSquared;
    const Vector3<T> p_XoBc_F = p_BoBc_F_ - p_BoXo_F;
    const Matrix3<T> Sp_BoBc_F = CrossProductMatrixSquared(p_BoBc_F_);
    const Matrix3<T> Sp_XoBc_F = CrossProductMatrixSquared(p_XoBc_F);
    I_Bo_F_.get_mutable_symmetric_matrix_view() =
        I_Bo_F_.get_matrix() + mass_ * (Sp_BoBc_F - Sp_XoBc_F);
    p_BoBc_F_ = p_XoBc_F;
    return *this;
  }

  /// This methods perfomrs the "parallel axis theorem" for spatial inertias:
  /// given this spatial inertia `M_Bo_F` about `Bo` and expressed in frame `F`,
  /// this method returns this spatial inertia to but computed about a new
  /// origin Xo. The result still is expressed in frame `F`.
  /// @see ShiftOriginInPlace() for the in-place operation.
  ///
  /// See Section 2.1.2, p. 20 of A. Jain's book.
  ///
  /// @param[in] p_BoXo_F Vector from the original origin `Bo` to the new origin
  /// `Xo`, expressed in the spatial inertia frame `F`.
  /// @returns `M_Xo_F` This same spatial inertia but computed about
  /// origin `Xo`.
  SpatialInertia ShiftOrigin(const Vector3<T>& p_BoXo_F) const {
    return SpatialInertia(*this).ShiftOriginInPlace(p_BoXo_F);
  }

 private:
  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // Mass of the body.
  T mass_{nan()};
  // Center of mass measured in X and expressed in Y. Typically X is the body
  // frame and Y is the world frame.
  Vector3<T> p_BoBc_F_{Vector3<T>::Constant(nan())};
  // Rotational inertia about Xo and expressed in Y.
  RotationalInertia<T> I_Bo_F_{};  // Defaults to NaN initialized inertia.
};

template <typename T> inline
std::ostream& operator<<(std::ostream& o,
                         const SpatialInertia<T>& M) {
  return o
      << " mass = " << M.get_mass() << std::endl
      << " com = [" << M.get_com().transpose() << "]^T" << std::endl
      << " I = " << std::endl
      << M.get_rotational_inertia();
}

}  // namespace multibody
}  // namespace drake
