#pragma once

#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/math/spatial_vector.h"

namespace drake {
namespace multibody {

#if 0
template <typename T>
class GeneralSpatialVector {
 public:
  enum {
    kSpatialVectorSize = 6,
    kSpatialVectorAngularSize = 3,
    kSpatialVectorLinearSize = 3
  };
  typedef Eigen::Matrix<T, kSpatialVectorSize, 1> CoeffsEigenType;

  GeneralSpatialVector() {}

  GeneralSpatialVector(const Vector3<T>& w, const Vector3<T>& v) {
    V_ << w, v;
  }

  /// Constructs a SpatialVector from an Eigen expression.
  /// This methods static asserts at compile time that the input expression
  /// @p v has a compile time size of six (6).
  template<typename Derived>
  GeneralSpatialVector(const Eigen::MatrixBase<Derived>& v) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 6, 1);
    V_ = v;
  }

  int size() const { return kSpatialVectorSize; }
  int angular_size() const { return kSpatialVectorAngularSize; }
  int linear_size() const { return kSpatialVectorLinearSize; }

  const CoeffsEigenType& get_coeffs() const { return V_;}

  T& operator[](int i) { return V_[i];}
  const T& operator[](int i) const { return V_[i];}

  const Vector3<T>& rotational() const {
    return *reinterpret_cast<const Vector3<T>*>(V_.data());
  }

  Vector3<T>& rotational() {
    return *reinterpret_cast<Vector3<T>*>(V_.data());
  }

  const Vector3<T>& translational() const {
    return *reinterpret_cast<const Vector3<T>*>(
        V_.data() + kSpatialVectorAngularSize);
  }

  Vector3<T>& translational() {
    return *reinterpret_cast<Vector3<T>*>(
        V_.data() + kSpatialVectorAngularSize);
  }

  T dot(const GeneralSpatialVector& V) const {
    return V_.dot(V.V_);
  }

  const T* data() const { return V_.data(); }

  T* mutable_data() { return V_.data(); }

  bool IsApprox(const GeneralSpatialVector<T>& other,
                double tolerance = Eigen::NumTraits<T>::epsilon()) {
    return translational().isApprox(other.translational(), tolerance) &&
           rotational().isApprox(other.rotational(), tolerance);
  }

  void SetZero() {
    V_.setZero();
  }

  void SetToNaN() {
   V_.setConstant(nan());
  }

 private:
  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  CoeffsEigenType V_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

template <typename T> inline
std::ostream& operator<<(std::ostream& o, const GeneralSpatialVector<T>& V) {
  o << "[" << V[0];
  for(int i = 1; i < V.size(); ++i) o << ", " << V[i];
  o << "]^T";  // The "transpose" symbol.
  return o;
}

/// Multiplication from the left by a scalar.
template <typename T>
inline GeneralSpatialVector<T> operator*(
    const T& s, const GeneralSpatialVector<T>& V)
{
  return GeneralSpatialVector<T>(s * V.rotational(), s * V.translational());
}
#endif

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

namespace internal {
// Helper traits-like struct to determine the number of columns at compile time
// of the internal Eigen representation of SpatialVelocityJacobian.
template <int ndofs>
struct SpatialVelocityJacobianMaxSize {
  enum { kMaxCols = ndofs};
};

// Specialization to Eigen::Dynamic. In this case we want a maximum fixed size
// of six (6) to avoid dynamic memory allocation for fast computations.
// See Eigen documentation here:
// http://eigen.tuxfamily.org/dox/classEigen_1_1Matrix.html#maxrows
template <>
struct SpatialVelocityJacobianMaxSize<Eigen::Dynamic> {
  enum { kMaxCols = 6};
};
}  // namespace internal

// Forward declaration of the SpatialVelocityJacobian's transpose.
template <typename T, int ndofs> class SpatialVelocityJacobianTranspose;

/// Class representing the Hinge matrix...
/// Essentially a matrix containing a SpatialVector in the column corresponding
/// to a generalized velocity.
template <typename T, int ndofs>
class SpatialVelocityJacobian {
 public:
  enum {
    kSpatialVectorSize = 6,
    kSpatialVectorAngularSize = 3,
    kSpatialVectorLinearSize = 3,
    kMaxCols = internal::SpatialVelocityJacobianMaxSize<ndofs>::kMaxCols
  };
  typedef Eigen::Matrix<T,
                        kSpatialVectorSize, ndofs, /*size*/
                        0, /*options*/
                        kSpatialVectorSize, kMaxCols> /*max size*/
      CoeffsEigenType;

  /// Default constructor. For a dynamic-size Jacobian the number of columns is
  /// initially zero. The method resize() can be called to allocate a given
  /// number of columns on a dynamic-size Jacobian. resize() however will
  /// trigger an assertion on fixed-size Jacobians.
  SpatialVelocityJacobian() {}

  /// This constructor is used for spatial velocity Jacobians of dynamic size.
  /// The input parameter @p num_velocities has no effect for fixed sized
  SpatialVelocityJacobian(int num_velocities) :
      J_(int(kSpatialVectorSize), num_velocities) {}

  /// Dynamically allocates @p num_columns for dynamic-size Jacobians.
  /// This method will trigger an assertion for fixed-size Jacobians.
  void resize(int num_columns) {
    J_.resize(kSpatialVectorSize, num_columns);
  }

  int rows() const { return J_.rows(); }
  int cols() const { return J_.cols(); }

  const T& operator()(int i, int j) const { return J_(i, j);}

  const GeneralSpatialVector<T>& col(int i) const {
    DRAKE_ASSERT(0 <= i && i < cols());
    // Assumes column major order. Eigen's default.
    return *reinterpret_cast<const GeneralSpatialVector<T>*>(
        J_.data() + kSpatialVectorSize * i);
  }

  GeneralSpatialVector<T>& col(int i) {
    DRAKE_ASSERT(0 <= i && i < cols());
    // Assumes column major order. Eigen's default.
    return *reinterpret_cast<GeneralSpatialVector<T>*>(
        J_.data() + kSpatialVectorSize * i);
  }

  SpatialVelocityJacobianTranspose<T, ndofs> transpose() const;

  const CoeffsEigenType& get_coeffs() const { return J_;}

  CoeffsEigenType& get_mutable_coeffs() { return J_;}

  static const SpatialVelocityJacobian& View(const T* data) {
    return *reinterpret_cast<const SpatialVelocityJacobian*>(data);
  }

  static SpatialVelocityJacobian& MutableView(T* data) {
    return *reinterpret_cast<SpatialVelocityJacobian*>(data);
  }

  const T* mutable_data() const { return J_.data(); }

  T* mutable_data() { return J_.data(); }

  bool IsApprox(const SpatialVelocityJacobian& other,
                double tolerance = Eigen::NumTraits<T>::epsilon()) {
    return get_coeffs().isApprox(other.get_coeffs(), tolerance);
  }

 private:
  CoeffsEigenType J_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

/// Operator multiplication of a rotation matrix @p R_AB times a
/// SpatialVelocityJacobian @p J_XY_B. This operator re-expresses the angular
/// and linear components of the Jacobian @p J_XY_B in a frame `A`.
/// Essentially for each column `V_XY_B` in `J_XY_B` this operator performs the
/// operation:
///   Vw_XY_A = R_AB * Vw_XY_B
///   Vv_XY_A = R_AB * Vv_XY_B
/// where:
///   Vw_XY = Vw_XY.rotational(), the angular component of the spatial vector.
///   Vv_XY = Vv_XY.translational() , the linear component of the spatial vector.
template <typename T, int ndofs>
inline SpatialVelocityJacobian<T, ndofs> operator*(
    const Matrix3<T>& R_AB, const SpatialVelocityJacobian<T, ndofs>& J_XY_B)
{
  SpatialVelocityJacobian<T, ndofs> J_XY_A;
  auto& Eigen_J_XY_A = J_XY_A.get_mutable_coeffs();
  const auto& Eigen_J_XY_B = J_XY_B.get_coeffs();

  // Re-express angular components.
  // TODO: have a SpatialVelocityJacobian::topRows (or similar name).
  Eigen_J_XY_A.template topRows<3>().noalias() =
      R_AB * Eigen_J_XY_B.template topRows<3>();

  // Re-express linear components.
  Eigen_J_XY_A.template bottomRows<3>().noalias() =
      R_AB * Eigen_J_XY_B.template bottomRows<3>();

  return J_XY_A;
};

/// @returns V_XY_E The spatial velocity of frame `Y` measured in frame `X`,
///                 expressed in frame `E`.
template <typename T, int ndofs>
inline GeneralSpatialVector<T> operator*(
    const SpatialVelocityJacobian<T, ndofs>& J_XY_E, const Vector<T, ndofs>& v)
{
  return GeneralSpatialVector<T>(J_XY_E.get_coeffs() * v);
};

template <typename T, int ndofs>
class SpatialVelocityJacobianTranspose {
 public:
  typedef Eigen::Transpose<
      typename SpatialVelocityJacobian<T, ndofs>::CoeffsEigenType>
      TransposeEigenType;
  typedef Eigen::Transpose<
      const typename SpatialVelocityJacobian<T, ndofs>::CoeffsEigenType>
      ConstTransposeEigenType;

  explicit SpatialVelocityJacobianTranspose(
      const SpatialVelocityJacobian<T, ndofs>& H) : H_(H) {}

  int rows() const { return H_.cols(); }
  int cols() const { return H_.rows(); }

  const T& operator()(int i, int j) const { return H_(j, i);}

  const ConstTransposeEigenType get_coeffs() const {
    return H_.get_coeffs().transpose();
  }

  TransposeEigenType get_mutable_coeffs() {
    return H_.get_coeffs().transpose();
  }

 private:
  const SpatialVelocityJacobian<T, ndofs>& H_;
};

template <typename T, int ndofs>
inline SpatialVelocityJacobianTranspose<T, ndofs>
SpatialVelocityJacobian<T, ndofs>::transpose() const {
  return SpatialVelocityJacobianTranspose<T, ndofs>(*this);
}


/// This defines a SpatialVelocityJacobian with a maximum number of columns at
/// compile time of six (6).
/// The biggest reason why you might want to use this class is to avoid dynamic
/// memory allocation.
template <typename T>
using SpatialVelocityJacobianUpTo6 = SpatialVelocityJacobian<T, Eigen::Dynamic>;

template <typename T>
using SpatialVelocityJacobianUpTo6Transpose =
SpatialVelocityJacobianTranspose<T, Eigen::Dynamic>;

/// This method multiplies the transpose of a Jacobian matrix Ja with another
/// Jacobian Jb. The input Jacobians Ja and Jb are expected to be of dynamic
/// size up-to a maximum of six and therefore the expected result of this
/// operation is a dynamic sized matrix with a fixed size at compile time of
/// up-to six rows and columns.
template <typename T>
inline MatrixUpTo6<T> operator*(
    const SpatialVelocityJacobianUpTo6Transpose<T>& Ja,
    const SpatialVelocityJacobianUpTo6<T>& Jb)
{
  return Ja.get_coeffs() * Jb.get_coeffs();
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
  /// Default constructor leaves the offset uninitialized resulting in a zero
  /// cost operation.
  ShiftOperator() {}

  /// Constructs a rigid body transformation operator between a pair of frames
  /// `X` and `Y` given a vector @p offset_XoYo_F from `Xo` to `Yo`.
  /// For a vector @p offset_XoYo_F expressed in a frame `F`, this operator can
  /// only operate on spatial vectors expressed in the same frame `F`.
  /// @param[in] offset_XoYo_F Vector from `Xo` to `Yo` expressed in an implicit
  ///                          frame `F`.
  ShiftOperator(const Vector3<T>& offset_XoYo_F) : offset_(offset_XoYo_F) {}

#if 0
  /// Constructs a rigid body transformation operator between a pair of frames
  /// `X` and `Y` given an arbitrary Eigen expression of a vector
  /// @p offset_XoYo_F from `Xo` to `Yo`. For a vector @p offset_XoYo_F
  /// expressed in a frame `F`, this operator can only operate on spatial
  /// vectors expressed in the same frame `F`.
  /// @param[in] offset_XoYo_F Vector from `Xo` to `Yo` expressed in an implicit
  ///                          frame `F`.
  template<typename Derived>
  ShiftOperator(const Eigen::MatrixBase<Derived>& offset_XoYo_F) {
    // Static asserts that EigenMatrix is of fixed size 3x1.
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 1);
    offset_ = offset_XoYo_F;
  }
#endif

  // Returns the vector from the origin of frame X to the origin of frame Y
  // expressed in the implicit frame F.
  const Vector3<T>& offset() const { return offset_; }

  ShiftOperatorTranspose<T> transpose() const;

  /// Creates a %ShiftOperator initialized with a NaN offset vector.
  static ShiftOperator<T> NaN() {
    return ShiftOperator<T>(Vector3<T>::Constant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN()));
  }

  /// Sets the offset of this operator to NaN. Typically used to quickly detect
  /// uninitialized values since NaN will trigger a chain of invalid
  /// computations that then can be tracked to the source.
  void SetToNaN() {
    offset_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

 private:
  Vector3<T> offset_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
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
  static GeneralSpatialVector<T> ShiftSpatialVelocity(
      const GeneralSpatialVector<T>& V_AB_E, const Vector3<T>& r_BQ_E) {
    return GeneralSpatialVector<T>(
        /* Same angular velocity. */
        V_AB_E.rotational(),
        /* Linear velocity v_AQ = v_AB + w_AB.cross(r_BQ). */
        V_AB_E.translational() + V_AB_E.rotational().cross(r_BQ_E));
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
inline GeneralSpatialVector<T> operator*(
    const ShiftOperatorTranspose<T>& phiT_BQ_E, const GeneralSpatialVector<T>& V_AB_E)
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

}  // namespace multibody
}  // namespace drake
