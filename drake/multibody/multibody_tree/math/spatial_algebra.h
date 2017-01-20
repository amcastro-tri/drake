#pragma once

#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/vector_space.h"

namespace drake {
namespace multibody {
namespace math {

/// Given the spatial velocity `V_AB` of a frame `B` with respect to a frame `A`,
/// compute the spatial velocity of a frame `Q` rigidly moving with `B` but
/// offset by vector `r_BA`.
/// @param[in] V_AB Spatial velocity of frame `B` with respect to frame `A`,
/// expressed in frame `A`.
/// @param[in] r_BQ_A Shift vector from `Bo` to `Qo`, expressed in frame `A`.
/// @returns V_AQ_A The spatial velocity of frame `Q` with respect to `A` and
/// expressed in frame `A`. The results is a matrix of size 6 x V_AB.cols() with
/// each column containing the shifted spatial velocity in the corresponing
/// column of `V_AB`.
template<class Derived>
static auto ShiftSpatialVelocity(
    const Eigen::MatrixBase<Derived> &V_AB,
    const Vector3<typename Derived::Scalar> &r_BQ_A) {
  // Essentially a row vector of spatial vectors with each spatial vector
  // arranged columnwise. Therefore a matrix of size 6 times V_AB.cols().
  Eigen::Matrix<typename Derived::Scalar, 6, Derived::ColsAtCompileTime>
      V_AQ_A(6, V_AB.cols());
  // Same angular velocity.
  V_AQ_A.template topRows<3>() = V_AB.template topRows<3>();
  // Linear velocity v_AQ = v_AB + w_AB.cross(r_BQ).
  V_AQ_A.template bottomRows<3>().noalias() =
      V_AB.template bottomRows<3>() +
          V_AB.template topRows<3>().colwise().cross(r_BQ_A);
  return V_AQ_A;
}

template<class Derived>
static auto ReExpressSpatialVelocity(
    const Eigen::MatrixBase<Derived> &V_AB_Y,
    const Matrix3<typename Derived::Scalar> &R_XY) {
  // Essentially a row vector of spatial vectors with each spatial vector
  // arranged columnwise. Therefore a matrix of size 6 times V_AB.cols().
  Eigen::Matrix<typename Derived::Scalar, 6, Derived::ColsAtCompileTime>
      V_AB_X(6, V_AB_Y.cols());
  V_AB_X.template topRows<3>().noalias() =
      R_XY * V_AB_Y.template topRows<3>();
  V_AB_X.template bottomRows<3>().noalias() =
      R_XY * V_AB_Y.template bottomRows<3>();
  return V_AB_X;
}

// SpatialVector traits.
namespace internal {
// Generic specialization for SpatialVector
template <class V>
struct traits<SpatialVector<V>> {
  typedef V Vec;
  enum {
    kNumDimensions = VectorSpace<V>::kNumDimensions,
    kSpatialVectorSize = VectorSpace<V>::kSpatialVectorSize,
    kSpatialVectorAngularSize = VectorSpace<V>::kSpatialVectorAngularSize,
    kSpatialVectorLinearSize = VectorSpace<V>::kSpatialVectorLinearSize
  };
};
}


template <typename T>
class SpatialVector {
 public:
  enum {
    kSpatialVectorSize = 6,
    kSpatialVectorAngularSize = 3,
    kSpatialVectorLinearSize = 3
  };
  typedef Eigen::Matrix<T, kSpatialVectorSize, 1> CoeffsEigenType;

  SpatialVector() {}

  SpatialVector(const Vector3<T>& w, const Vector3<T>& v) {
    V_ << w, v;
  }

  int size() const { return kSpatialVectorSize; }
  int angular_size() const { return kSpatialVectorAngularSize; }
  int linear_size() const { return kSpatialVectorLinearSize; }

  const CoeffsEigenType& get_coeffs() const { return V_;}

  T& operator[](int i) { return V_[i];}
  const T& operator[](int i) const { return V_[i];}

  const Vector3<T>& angular() const {
    return *reinterpret_cast<const Vector3<T>*>(V_.data());
  }

  Vector3<T>& angular() {
    return *reinterpret_cast<Vector3<T>*>(V_.data());
  }

  const Vector3<T>& linear() const {
    return *reinterpret_cast<const Vector3<T>*>(
        V_.data() + kSpatialVectorAngularSize);
  }

  Vector3<T>& linear() {
    return *reinterpret_cast<Vector3<T>*>(
        V_.data() + kSpatialVectorAngularSize);
  }

 private:
  CoeffsEigenType V_;
};

template <typename T> inline
std::ostream& operator<<(std::ostream& o, const SpatialVector<T>& V) {
  o << "[" << V[0];
  for(int i = 1; i < V.size(); ++i) o << ", " << V[i];
  o << "]^T";  // The "transpose" symbol.
  return o;
}

/// Multiplication from the left by a scalar.
template <typename T>
inline SpatialVector<T> operator*(
    const T& s, const SpatialVector<T>& V)
{
  return SpatialVector<T>(s * V.angular(), s * V.linear());
}

/// Class representing the Hinge matrix...
/// Essentially a matrix containing a SpatialVector in the column corresponding
/// to a generalized velocity.
template <typename T, int ndofs>
class SpatialVelocityJacobian {
 public:
  enum {
    kSpatialVectorSize = 6,
    kSpatialVectorAngularSize = 3,
    kSpatialVectorLinearSize = 3
  };
  typedef Eigen::Matrix<T, kSpatialVectorSize, ndofs> CoeffsEigenType;

  int rows() const { return J_.rows(); }
  int cols() const { return J_.cols(); }

  const T& operator()(int i, int j) const { return J_(i, j);}

  const SpatialVector<T>& col(int i) const {
    DRAKE_ASSERT(0 < i && i < cols());
    // Assumes column major order. Eigen's default.
    return *reinterpret_cast<const SpatialVector<T>*>(&(J_.col(i)[0]));
  }

  SpatialVector<T>& col(int i) {
    DRAKE_ASSERT(0 < i && i < cols());
    // Assumes column major order. Eigen's default.
    return *reinterpret_cast<SpatialVector<T>*>(&(J_.col(i)[0]));
  }

  const CoeffsEigenType& get_coeffs() const { return J_;}

  CoeffsEigenType& get_mutable_coeffs() { return J_;}

 private:
  CoeffsEigenType J_;
};

// Forward declaration of the ShiftOperator's transpose.
template <typename T> class ShiftOperatorTranspose;

/// A class representing the Rigid Body Transformation Matrix as defined in
/// Section 1.4 of A Jain's book.
/// Given a pair of frames `x` and `y`
///
/// @tparam T
template <typename T>
class ShiftOperator {
 public:
  /// Constructs a rigid body transformation operator between a pair of frames
  /// `X` and `Y` given a vector @p l from `Xo` to `Yo`. For a vector @p l_F
  /// expressed in a frame `F`, this operator can only operate on spatial
  /// vectors expressed in the same frame `F`.
  /// @param[in] l_F Vector from `Xo` to `Yo` expressed in an implicit
  /// frame `F`.
  ShiftOperator(const Vector3<T> offset_XoYo_F) : offset_(offset_XoYo_F) {}

  // Returns the vector from the origin of frame X to the origin of frame Y
  // expressed in the implicit frame F.
  const Vector3<T>& offset() const { return offset_; }

  ShiftOperatorTranspose<T> transpose() const;

 private:
  Vector3<T> offset_;
};

template <typename T>
class ShiftOperatorTranspose {
public:
  explicit ShiftOperatorTranspose(const ShiftOperator<T>& phi_XY_F) :
      phi_(phi_XY_F) {}

  // Returns the vector from the origin of frame X to the origin of frame Y
  // expressed in the implicit frame F.
  const Vector3<T>& offset() const { return phi_.offset(); }

  /// Given the spatial velocity `V_AB` of a frame `B` with respect to a frame 
  /// `A`, compute the spatial velocity of a frame `Q` rigidly moving with `B` 
  /// but offset by vector `r_BQ`.
  ///
  /// The operation performed, in vector free form, is:
  ///   w_AQ = w_AB,  i.e. the angular velocity of frame B and Q is the same.
  ///   v_AQ = v_AB + w_AB x r_BQ
  ///
  /// All quantities above must be expressed in the same frame `E` i.e:
  ///   w_AQ_E = w_AB_E
  ///   v_AQ_E = v_AB_E + w_AB_E x r_BQ_E
  ///
  /// @param[in] V_AB_E Spatial velocity of frame `B` with respect to frame `A`,
  /// expressed in frame `E`.
  /// @param[in] r_BQ_E Shift vector from `Bo` to `Qo`, expressed in frame `E`.
  /// @returns V_AQ_E The spatial velocity of frame `Q` with respect to `A` and
  /// expressed in frame `A`.
  static SpatialVector<T> ShiftSpatialVelocity(
      const SpatialVector<T>& V_AB_E, const Vector3<T>& r_BQ_E) {
    return SpatialVector<T>(
        /* Same angular velocity. */
        V_AB_E.angular(),
        /* Linear velocity v_AQ = v_AB + w_AB.cross(r_BQ). */
        V_AB_E.linear() + V_AB_E.angular().cross(r_BQ_E));
  }

 private:
  const ShiftOperator<T>& phi_;
};

template <typename T>
inline ShiftOperatorTranspose<T> ShiftOperator<T>::transpose() const {
  return ShiftOperatorTranspose<T>(*this);
}

/// Given the spatial velocity `V_AB` of a frame `B` measured in a frame `A`, 
/// compute the spatial velocity of a frame `Q` rigidly moving with `B` 
/// but offset by vector `r_BQ`.
///
/// The operation performed, in vector free form, is:
///   V_AQ = phi_BQ^T * V_AB
///
/// All quantities above must be expressed in the same frame `E` i.e:
///   V_AQ_E = phi_BQ_E^T * V_AB_E
///
/// @param[in] phiT_BQ_E Transpose of the shift operator from frame `B` to `Q`
/// expressed in a frame `E`.
/// @param[in] V_AB_E The spatial velocity of frame `B` measured in `A` 
/// and expressed in `E`.
/// @returns V_AQ_E The spatial velocity of frame `Q` measured in `A` and 
/// expressed in `E`.
template <typename T>
inline SpatialVector<T> operator*(
    const ShiftOperatorTranspose<T>& phiT_BQ_E, const SpatialVector<T>& V_AB_E)
{
  return ShiftOperatorTranspose<T>::ShiftSpatialVelocity(
      V_AB_E, phiT_BQ_E.offset());
}

/// @param[in] phiT_AB_F Transpose of the shift operator from frame A to B
/// expressed in a frame F.
/// @param[in] H_FA A spatial velocity Jacobian with each column representing a
/// spatial velocity of frame A measured and expressed in frame F.
/// @returns H_FB The spatial velocity jacobain for frame B measured and
/// expressed in F.
template <typename T, int ndofs>
inline SpatialVelocityJacobian<T, ndofs> operator*(
    const ShiftOperatorTranspose<T>& phiT_BQ_E,
    const SpatialVelocityJacobian<T, ndofs>& J_AB_E)
{
  SpatialVelocityJacobian<T, ndofs> J_AQ_E;
  auto& Eigen_J_AQ_E = J_AQ_E.get_mutable_coeffs();
  const auto& Eigen_J_AB_E = J_AB_E.get_coeffs();
  // Same angular velocity.
  Eigen_J_AQ_E.template topRows<3>() = Eigen_J_AB_E.template topRows<3>();
  // Linear velocity v_AQ = v_AB + w_AB.cross(r_BQ).
  Eigen_J_AQ_E.template bottomRows<3>().noalias() =
      Eigen_J_AB_E.template bottomRows<3>() +
          Eigen_J_AB_E.template topRows<3>().colwise().cross(
              phiT_BQ_E.offset());
  return J_AQ_E;
}

template <typename T, int ndofs> inline
std::ostream& operator<<(std::ostream& o,
                         const SpatialVelocityJacobian<T, ndofs>& J) {
  for(int i = 0; i < J.rows(); ++i) {
    o << "[" << J(i, 0);
    for (int j = 1; j < J.cols(); ++j) o << " | " << J(i, j);
    o << "]" << std::endl;
  }
  return o;
}

}  // math
}  // namespace multibody
}  // namespace drake
