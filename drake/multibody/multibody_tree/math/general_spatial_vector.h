#pragma once

#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"

namespace drake {
namespace multibody {

template <typename T>
class GeneralSpatialVector : public SpatialVector<GeneralSpatialVector, T> {
  // We need the fully qualified class name below for the clang compiler to
  // work. Without qualifiers the code is legal according to the C++11 standard
  // but the clang compiler still gets confused. See:
  // http://stackoverflow.com/questions/17687459/clang-not-accepting-use-of-template-template-parameter-when-using-crtp
  typedef SpatialVector<::drake::multibody::GeneralSpatialVector, T> Base;

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeneralSpatialVector)

  /// Default constructor. In Release builds the elements of the newly
  /// constructed spatial velocity are left uninitialized resulting in a zero
  /// cost operation. However in Debug builds those entries are set to NaN so
  /// that operations using this uninitialized spatial velocity fail fast,
  /// allowing fast bug detection.
  GeneralSpatialVector() : Base() {}

  /// SpatialVelocity constructor from an angular velocity @p w and a linear
  /// velocity @p v.
  //GeneralSpatialVector(const Eigen::Ref<const Vector3<T>>& w,
  //                const Eigen::Ref<const Vector3<T>>& v) : Base(w, v) {}

  GeneralSpatialVector(const Vector3<T>& w, const Vector3<T>& v) : Base(w, v) {}

  /// SpatialVelocity constructor from an Eigen expression that represents a
  /// six-dimensional vector.
  /// This constructor will assert the size of V is six (6) at compile-time
  /// for fixed sized Eigen expressions and at run-time for dynamic sized Eigen
  /// expressions.
  template <typename Derived>
  explicit GeneralSpatialVector(const Eigen::MatrixBase<Derived>& V) : Base(V) {}

  /// In-place shift of a %SpatialVelocity from one point on a rigid body
  /// or frame to another point on the same body or frame.
  /// `this` spatial velocity `V_ABp_E` of a frame B at a point P fixed
  /// on B, measured in a frame A, and expressed in a frame E, is
  /// modified to become `V_ABq_E`, representing the velocity of another
  /// point Q on B instead (see class comment for more about this
  /// notation). This requires adjusting the translational (linear) velocity
  /// component to account for the velocity difference between P and Q
  /// due to the angular velocity of B in A.
  ///
  /// We are given the vector from point P to point Q, as a position
  /// vector `p_BpBq_E` (or `p_PQ_E`) expressed in the same frame E as the
  /// spatial velocity. The operation performed, in coordinate-free form, is:
  /// <pre>
  ///   w_AB  = w_AB,  i.e. the angular velocity is unchanged.
  ///   v_ABq = v_ABp + w_AB x p_BpBq
  /// </pre>
  /// where w and v represent the angular and linear velocity components
  /// respectively.
  ///
  /// For computation, all quantities above must be expressed in a common
  /// frame E; we add an `_E` suffix to each symbol to indicate that.
  ///
  /// This operation is performed in-place modifying the original object.
  ///
  /// @param[in] p_BpBq_E
  ///   Shift vector from point P of body B to point Q of B,
  ///   expressed in frame E. The "from" point `Bp` must be the point
  ///   whose velocity is currently represented in this spatial velocity,
  ///   and E must be the same expressed-in frame as for this spatial
  ///   velocity.
  ///
  /// @returns A reference to `this` spatial velocity which is now `V_ABq_E`,
  ///   that is, the spatial velocity of frame B at point Q, still
  ///   measured in frame A and expressed in frame E.
  ///
  /// @see Shift() to compute the shifted spatial velocity without modifying
  ///      this original object.
  GeneralSpatialVector<T>& ShiftInPlace(const Vector3<T>& p_BQ_E) {
    this->translational() += this->rotational().cross(p_BQ_E);
    return *this;
  }

  /// Shift of a %SpatialVelocity from one point on a rigid body
  /// or frame to another point on the same body or frame.
  /// This is an alternate signature for shifting a spatial velocity's
  /// point that does not change the original object. See
  /// ShiftInPlace() for more information.
  ///
  /// @param[in] p_BpBq_E
  ///   Shift vector from point P of body B to point Q of B,
  ///   expressed in frame E. The "from" point `Bp` must be the point
  ///   whose velocity is currently represented in this spatial velocity,
  ///   and E must be the same expressed-in frame as for this spatial
  ///   velocity.
  ///
  /// @retval V_ABq_E
  ///   The spatial velocity of frame B at point Q, measured in frame
  ///   A and expressed in frame E.
  ///
  /// @see ShiftInPlace() to compute the shifted spatial velocity in-place
  ///      modifying the original object.
  GeneralSpatialVector<T> Shift(const Vector3<T>& p_BQ_E) const {
    return GeneralSpatialVector<T>(*this).ShiftInPlace(p_BQ_E);
  }

  T dot(const GeneralSpatialVector<T>& V) const {
    return this->get_coeffs().dot(V.get_coeffs());
  }
};

template <typename T>
inline GeneralSpatialVector<T> operator+(
    const GeneralSpatialVector<T>& Va, const GeneralSpatialVector<T>& Vb)
{
  return GeneralSpatialVector<T>(Va.get_coeffs() + Vb.get_coeffs());
}

}  // namespace multibody
}  // namespace drake
