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

template <typename T>
class SpatialInertia {
 public:
  /// Default SpatialInertia constructor initializes mass, center of mass and
  /// rotational inertia to invalide NaN's for a quick detection of
  /// un-initialized values.
  SpatialInertia() {}

  /// Constructs a spatial inertia from a given mass, center of mass and
  /// rotational inertia. The center of mass is measured from an origin Xo, i.e.
  /// the center of mass is a vector from origin `Xo` to the body's center of
  /// mass `Bc`, `com = p_XoBc`. The rotational inertia is exptected to be
  /// computed about the same origin `Xo`.
  /// Both center of mass `com` and rotational inertia `I` are expected to be
  /// expressed on a same frame `Y`.
  /// Typically frame `X` is the body frame `B` while a typical frame `Y` is
  /// the world frame `W`.
  /// @param[in] mass The mass of the body.
  /// @param[in] com The center of mass of the body measured from an origin Xo
  /// and expressed in a frame Y.
  /// @param[in] I Rotational inertia of the body computed about origin Xo and
  /// expressed in a frame Y.
  SpatialInertia(
      const T& mass, const Vector3<T>& com, const RotationalInertia<T>& I) :
      mass_(mass), p_XoBc_Y_(com), I_Xo_Y_(I) {}

  // Default copy constructor and copy assignment.
  SpatialInertia(const SpatialInertia<T>& other) = default;
  SpatialInertia& operator=(const SpatialInertia<T>& other) = default;

  const T& get_mass() const { return mass_;}

  const Vector3<T>& get_com() const { return p_XoBc_Y_;}

  const RotationalInertia<T>& get_rotational_inertia() const { return I_Xo_Y_;}

  /// Given this spatial inertia `M_Bo_F` about `Bo` and expressed in frame `F`,
  /// this method re-expresses the same spatial inertia in another frame `A`.
  /// This operation is performed in-place modifying the original object.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns A references to `this` object which now is the same spatial inertia
  /// about `Bo` but expressed in frame `A`.
  SpatialInertia& ReExpressIn(const Matrix3<T>& R_AF) {
    p_XoBc_Y_ = R_AF * p_XoBc_Y_;  // Re-express com in A.
    I_Xo_Y_.ReExpressIn(R_AF);
    return *this;
  }

  /// Given this spatial inertia `M_Bo_F` about `Bo` and expressed in frame `F`,
  /// this method coputes the same spatial inertia but re-expressed in another
  /// frame `A`.
  /// @param[in] R_AF Rotation matrix from frame `F` to frame `A`.
  /// @returns M_Bo_A The same spatial inertia about `Bo` but expressed in
  /// frame `A`.
  SpatialInertia ReExpressedIn(const Matrix3<T>& R_AF) const {
    return SpatialInertia(*this).ReExpressIn(R_AF);
  }

#if 0
  SpatialInertia ReExpressIn(const Matrix3<T>& R_AF) {
    SpatialInertia<T> I_Bo_A;
    I_Bo_A = R_AF *
        I_Bo_F_.template selfadjointView<TriangularViewInUse>() *
        R_AF.transpose();
    return I_Bo_A;
  }
#endif

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
  Vector3<T> p_XoBc_Y_{Vector3<T>::Constant(nan())};
  // Rotational inertia about Xo and expressed in Y.
  RotationalInertia<T> I_Xo_Y_{};  // Defaults to NaN initialized inertia.
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
