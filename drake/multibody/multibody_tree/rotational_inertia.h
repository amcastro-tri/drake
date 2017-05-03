#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {

/// This class helps describe the mass distribution (inertia properties) of a
/// body or composite body about a particular point.  Herein, "composite body"
/// means one body or a collection of bodies that are welded together.  In this
/// documentation, "body" and "composite body" are used interchangeably.
///
/// A **rigid** body's mass distribution is described by three quantities:
/// the body's mass; the body's center of mass; and the body's rotational
/// inertia about a particular point. The term **rotational inertia** is used
/// here and by [Jain 2010] to distinguish from a body's **spatial inertia**.
/// In this class, a 3x3 **inertia matrix** I represents a body's rotational
/// inertia about a point and expressed in a frame (e.g., about-point P and
/// expressed-in-frame E with right-handed orthogonal unit vectors x̂, ŷ, ẑ).
/// <pre>
///     | Ixx Ixy Ixz |
/// I = | Ixy Iyy Iyz |
///     | Ixz Iyz Izz |
/// </pre>
/// The moments of inertia Ixx, Iyy, Izz and products of inertia Ixy, Ixz, Iyz
/// are defined in terms of the mass dm of a differential volume of the body.
/// The position of dm from about-point P is xx̂ + yŷ + zẑ = [x, y, z]_E.
/// <pre>
/// Ixx = ∫ (y² + z²) dm
/// Iyy = ∫ (x² + z²) dm
/// Izz = ∫ (x² + y²) dm
/// Ixy = - ∫ x y dm
/// Ixz = - ∫ x z dm
/// Iyz = - ∫ y z dm
/// </pre>
/// We use the negated convention for products of inertia, so that I serves
/// to relate angular velocity ω and angular momentum h via `h = I ⋅ ω`.
/// Ensure your products of inertia follow this negative sign convention.
///
/// The 3x3 inertia matrix is symmetric and its diagonal elements (moments of
/// inertia) and off-diagonal elements (products of inertia) are associated
/// with a body (or composite body) S, an about-point P, and an expressed-in-
/// frame E (x̂, ŷ, ẑ).  A rotational inertia is ill-defined unless there is a
/// body S, about-point P, and expressed-in-frame E. The user of this class is
/// responsible for tracking the body S, about-point P and expressed-in-frame E
/// (none of these are stored in this class). The rotational inertia class only
/// stores data for the inertia matrix, whose elements are initially set to NaN
/// to aid in ensuring only the lower-triangular part of the matrix is used
/// (the upper-triangular part is not used - with each element as NaN).
///
/// @note This class does not store the about-point nor the expressed-in-frame,
/// nor does this class help enforce consistency of the about-point or
/// expressed-in-frame. To help users of this class track the about-point and
/// expressed-in-frame. We strongly recommend the following notation.
///
/// In typeset material, use the symbol @f$ [I^{S/P}]_E @f$ to represent the
/// rotational inertia (inertia matrix) of a body (or composite body) S
/// about-point P, expressed in frame E. In code and comments, use the monogram
/// notation `I_SP_E` (e.g., as described in @ref multibody_spatial_inertia).
/// If the about-point P is fixed to a body B, the point is named @f$ B_P @f$
/// and this appears in code/comments as `Bp`.  Examples: `I_BBp_E` is rigid
/// body B's rotational inertia about-point Bp expressed-in-frame E; I_BBo_E is
/// B's rotational inertia about-point `Bo` (body B's origin) expressed-in-frame
/// E; and I_BBcm_E is B's inertia matrix about-point `Bcm` (B's center of
/// mass) expressed-in-frame E.
///
/// @note The rotational inertia (inertia matrix) can be re-expressed in terms
/// of a special frame whose orthogonal unit vectors are parallel to **principal
/// axes of inertia** so that the inertia matrix is diagonalized with elements
/// called **principal moments of inertia**.
///
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar.
/// Various methods in this class require numerical (not symbolic) data types.
template <typename T>
class RotationalInertia {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RotationalInertia)

  /// Constructs a rotational inertia that has all its moments/products of
  /// inertia equal to NaN (helps quickly detect uninitialized values).
  RotationalInertia() {}

  /// Constructs a rotational inertia with equal moments of inertia along its
  /// diagonal and with each product of inertia set to zero. This constructor
  /// is useful for the rotational inertia of a uniform-density sphere or cube.
  /// In debug builds, throws std::logic_error if I_triaxial is negative/NaN.
  explicit RotationalInertia(const T& I_triaxial) :
      RotationalInertia(I_triaxial, I_triaxial, I_triaxial, 0, 0, 0) {}

  /// Creates a rotational inertia with moments of inertia `Ixx`, `Iyy`, `Izz`,
  /// and with each product of inertia set to zero.
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// constructed from these arguments violate CouldBePhysicallyValid().
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz) :
      RotationalInertia(Ixx, Iyy, Izz, 0, 0, 0) {}

  /// Creates a rotational inertia with moments of inertia `Ixx`, `Iyy`, `Izz`,
  /// and with products of inertia `Ixy`, `Ixz`, `Iyz`.
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// constructed from these arguments violate CouldBePhysicallyValid().
  RotationalInertia(const T& Ixx, const T& Iyy, const T& Izz,
                    const T& Ixy, const T& Ixz, const T& Iyz) {
    set_moments_and_products_no_validity_check(Ixx, Iyy, Izz,
                                               Ixy, Ixz, Iyz);
    DRAKE_ASSERT_VOID(AssertCouldBeValid());
  }

  /// Constructs a rotational inertia for a particle Q of mass `mass`, whose
  /// position vector from about-point P is p_PQ_E = xx̂ + yŷ + zẑ = [x, y, z]_E,
  /// where E is the expressed-in-frame.
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// constructed from these arguments violate CouldBePhysicallyValid().
  /// @param mass The mass of particle Q.
  /// @param p_PQ_E Position from about-point P to Q, expressed-in-frame E.
  /// @return I_QP_E, Q's rotational inertia about-point Q expressed-in-frame E.
  /// @remark Negating the position vector p_PQ_E has no affect on the result.
  RotationalInertia(const T& mass, const Vector3<T>& p_PQ_E) :
      RotationalInertia(mass * p_PQ_E, p_PQ_E) {}

  /// Creates a rotational inertia for a unit-mass particle Q (mass = 1), whose
  /// position vector from about-point P is p_PQ_E = xx̂ + yŷ + zẑ = [x, y, z]_E,
  /// where E is the expressed-in-frame.
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// constructed from these arguments violate CouldBePhysicallyValid().
  /// @param p_PQ_E Position from about-point P to Q, expressed-in-frame E.
  /// @return I_QP_E, Q's rotational inertia about-point Q expressed-in-frame E.
  /// @remark Negating the position vector p_PQ_E has no affect on the result.
  static RotationalInertia<T> CalcUnitMassRotationalInertia(
      const Vector3<T>& p_PQ_E) {
    return RotationalInertia(p_PQ_E, p_PQ_E);
  }

  /// For consistency with Eigen's API, the rows() and cols() methods both
  /// return 3 (i.e., the number of rows or columns in the inertia matrix).
  int rows() const { return 3;}
  int cols() const { return 3;}

  /// Returns 3-element vector with moments of inertia [Ixx, Iyy, Izz].
  Vector3<T> get_moments() const { return I_SP_E_.diagonal(); }

  /// Returns 3-element vector with products of inertia [Ixy, Ixz, Iyz].
  Vector3<T> get_products() const {
    // Let operator(int ,int) decide whether upper/lower part of matrix is used.
    const RotationalInertia& Iref = *this;
    return Vector3<T>(Iref(0, 1), Iref(0, 2), Iref(1, 2));
  }

  /// Returns a rotational inertia's trace (i.e., Ixx + Iyy + Izz, the sum of
  /// the diagonal elements of the inertia matrix).  The trace happens to be
  /// invariant to its expressed-in-frame (i.e., the trace does not depend
  /// on the frame in which it is expressed).  The trace is useful because the
  /// largest moment of inertia Imax must have  trace / 3 <= Imax <= trace / 2,
  /// and the largest possible product of inertia must be <= Imax / 2.
  /// Hence, trace / 3 and trace / 2 give a lower and upper bound on the largest
  /// possible element that can be in a valid rotational inertia.
  T CalcTrace() const { return I_SP_E_.trace(); }

  /// Returns the theoretical maximum moment of inertia for `this` rotational
  /// inertia about-point P for *any* expressed-in-frame E. This maximum is
  /// 1/2 `this` rotational inertia's trace, i.e., (Ixx + Iyy + Izz) / 2.
  /// The trace happens to be invariant to its expressed-in-frame.
  /// @remark For a valid rotational inertia, the largest moment of inertia Imax
  /// has the range:  trace / 3 <= Imax <= trace / 2.
  T CalcTheoreticalMaximumMomentOfInertia() const {
    using std::abs;
    return 0.5 * abs(CalcTrace());
  }

  /// Const access to the `(i, j)` element of this rotational inertia.  Since
  /// rotational inertia is a symmetric matrix and the upper-off diagonal is not
  /// stored, this operator maps `(i, j)` to the appropriate memory location.
  /// This operator is meant for convenience (not speed) and helps avoid access
  /// to invalid or NaN parts of the inertia matrix.  To prevent a user from
  /// constructing a non-symmetric or non-physical rotational inertia element-
  /// by-element, a mutable version of this operator is not provided.
  const T& operator()(int i, int j) const {
    // Overwrites local copies of i and j.
    check_and_swap(&i, &j);
    return I_SP_E_(i, j);
  }

  /// Get a full 3x3 matrix copy of this rotational inertia.  The returned copy
  /// is symmetric and includes both lower and upper parts of the matrix.
  Matrix3<T> CopyToFullMatrix3() const { return get_symmetric_matrix_view(); }

  /// Returns this rotational inertia's Frobenius norm, which for a rotational
  /// inertia I having moments of inertia m and products of inertia p is:
  ///  ‖I‖F = sqrt(‖m‖₂² + 2 ‖p‖₂²), with sqrt() the square root function and
  ///                                ‖⋅‖₂ the ℓ²-norm of a vector.
  T Norm() const {
    using std::sqrt;
    return sqrt(get_moments().squaredNorm() + 2 * get_products().squaredNorm());
  }

  /// Compares two rotational inertias to determine if the differences in their
  /// moments of inertia (and similarly products of inertia) are within
  /// epsilon = `precision` * I_maximum (the minimum of I_maxA and I_maxB).
  /// I_maxA = abs(trace(`this')) / 2 is the largest value that can appear in a
  /// valid `this` rotational inertia (independent of the expressed-in-frame E).
  /// Similarly, I_maxB = abs(trace(`other`)) / 2.
  /// @param  precision is a small real positive number that multiplies
  ///         `I_maximum` to calculate epsilon = `precision` * `I_maximum`.
  ///         `precision` is usually based on two factors, namely the accuracy
  ///         of moments/products of inertia and/or machine-precision.
  /// @return IsApproxMomentsAndProducts(other, epsilon).
  /// @note: This method only works if all moments of inertia with scalar type T
  ///    in `this` and `other` can be converted to a double (discarding
  ///    supplemental scalar data such as derivatives of an AutoDiffScalar).
  ///    It fails at runtime if type T cannot be converted to `double`.
  /// @see IsApproxMomentsAndProducts().
  bool IsApproxEqualBasedOnMaximumPossibleMomentOfInertia(
      const RotationalInertia& other, const double precision) const {
    using std::min;
    const T I_maximumA = CalcTheoreticalMaximumMomentOfInertia();
    const T I_maximumB = other.CalcTheoreticalMaximumMomentOfInertia();
    const T I_maximum = min(I_maximumA, I_maximumB);
    const double epsilon = precision * ExtractDoubleOrThrow(I_maximum);
    return IsApproxMomentsAndProducts(other, epsilon);
  }

  /// Adds a rotational inertia `I_BP_E` to `this` rotational inertia.
  /// This method requires both rotational inertias (`I_BP_E` and `this`)
  /// to have the same about-point P and the same expressed-in-frame E.
  /// The += operator updates `this` so `I_BP_E' is added to `this'.
  /// @param I_BP_E Rotational inertia of a body (or composite body) B to
  ///        be added to `this` rotational inertia.  `I_BP_E` and `this`
  ///        must have the same about-point P and expressed-in-frame E.
  /// @return A reference to `this` rotational inertia. `this' changes
  ///         since rotational inertia `I_BP_E` has been added to it.
  /// @see operator+().
  RotationalInertia<T>& operator+=(const RotationalInertia<T>& I_BP_E) {
    this->get_mutable_triangular_view() += I_BP_E.get_matrix();
    return *this;
  }

  /// Adds a rotational inertia `I_BP_E` to `this` rotational inertia.
  /// This method requires both rotational inertias (`I_BP_E` and `this`)
  /// to have the same about-point P and the same expressed-in-frame E.
  /// @param I_BP_E Rotational inertia of a body (or composite body) B to
  ///        be added to `this` rotational inertia.  `I_BP_E` and `this`
  ///        must have the same about-point P and expressed-in-frame E.
  /// @return The sum of `this` rotational inertia and `I_BP_E'.
  /// @see operator+=().
  RotationalInertia<T> operator+(const RotationalInertia<T>& I_BP_E) const {
    return RotationalInertia(*this) += I_BP_E;
  }

  /// Subtracts a rotational inertia `I_BP_E` from `this` rotational inertia.
  /// This method requires both rotational inertias (`I_BP_E` and `this`)
  /// to have the same about-point P and the same expressed-in-frame E.
  /// The -= operator updates `this` so `I_BP_E' is subtracted from `this'.
  /// In debug builds, this method throws an exception if it violates
  /// CouldBePhysicallyValid().
  /// @param I_BP_E Rotational inertia of a body (or composite body) B to
  ///        be subtracted from `this` rotational inertia. `I_BP_E` and `this`
  ///        must have the same about-point P and expressed-in-frame E.
  /// @return A reference to `this` rotational inertia. `this' changes
  ///         since rotational inertia `I_BP_E` has been subtracted from it.
  /// @see operator-().
  /// @warning This operator may produce an invalid rotational inertia.
  ///    Use CouldBePhysicallyValid() to perform necessary (but insufficient)
  ///    checks on the physical validity of the resulting rotational inertia.
  ///    Sufficient conditions require calling CouldBePhysicallyValid() when the
  ///    about-point is the body's center of mass.
  /// @note This subtract operator is useful for computing rotational inertia
  /// of a body with a hole.  First the rotational inertia of a fully solid
  /// body S (without the hole) is calculated, then the rotational inertia of
  /// the hole (treated as a massive solid body B) is calculated. The rotational
  /// inertia of a composite body C (comprised of S and -B) is computed by
  /// subtracting B's rotational inertia from S's rotational inertia.
  /// @see MinusEqualsUnchecked().
  RotationalInertia<T>& operator-=(const RotationalInertia<T>& I_BP_E) {
    MinusEqualsUnchecked(I_BP_E);
    DRAKE_ASSERT_VOID(AssertCouldBeValid());
    return *this;
  }

  /// Subtracts a rotational inertia `I_BP_E` from `this` rotational inertia.
  /// This method requires both rotational inertias (`I_BP_E` and `this`)
  /// to have the same about-point P and the same expressed-in-frame E.
  /// In Debug builds, this method throws an exception if it violates
  /// CouldBePhysicallyValid().
  /// @param I_BP_E Rotational inertia of a body (or composite body) B to
  ///        be subtracted from `this` rotational inertia. `I_BP_E` and `this`
  ///        must have the same about-point P and expressed-in-frame E.
  /// @return The subtraction of `I_BP_E` from `this` rotational inertia.
  /// @see operator-=().
  /// @warning See warning and documentation for operator-=().
  RotationalInertia<T> operator-(const RotationalInertia<T>& I_BP_E) const {
    return RotationalInertia(*this) -= I_BP_E;
  }

  /// Multiplies `this` rotational inertia by a nonnegative scalar.
  /// In debug builds, throws std::logic_error if `nonnegative_scalar` < 0.
  /// @param nonnegative_scalar Nonnegative scalar which multiplies `this`.
  /// @return A reference to `this` rotational inertia. `this' changes
  ///         since `this` has been multiplied by `nonnegative_scalar'.
  /// @see operator*().
  RotationalInertia<T>& operator*=(const T& nonnegative_scalar) {
    DRAKE_ASSERT_VOID(AssertMultiplyByNonnegativeScalar(nonnegative_scalar));
    this->get_mutable_triangular_view() *= nonnegative_scalar;
    return *this;
  }

  /// Divides `this` rotational inertia by a positive scalar.
  /// In debug builds, throws std::runtime_error if `positive_scalar` <= 0.
  /// @param positive_scalar Positive scalar (> 0) which multiplies `this`.
  /// @return A reference to `this` rotational inertia. `this' changes
  ///         since `this` has been divided by `positive_scalar'.
  /// @see operator/(), operator*().
  RotationalInertia<T>& operator/=(const T& positive_scalar) {
    DRAKE_ASSERT(positive_scalar > 0);
    this->get_mutable_triangular_view() /= positive_scalar;
    return *this;
  }

  /// Divides `this` rotational inertia by a positive scalar.
  /// In debug builds, throws std::runtime_error if `positive_scalar` <= 0.
  /// constructed from these arguments violate CouldBePhysicallyValid().
  /// @param positive_scalar Positive scalar (> 0) which multiplies `this`.
  /// @return `this` rotational inertia divided by `positive_scalar`.
  /// @see operator/=(), operator*=().
  RotationalInertia<T> operator/(const T& positive_scalar) const {
    return RotationalInertia(*this) /= positive_scalar;
  }

  /// Calculates the dot-product of `this` rotational inertia with vector w_E.
  /// Both `this` and vector `w_E` must have the same expressed-in-frame E.
  /// @param w_E Vector to post-dot-multiply with `this` rotational inertia.
  /// @return The dot-product of `this` rotational inertia with `w_E`.
  Vector3<T> operator*(const Vector3<T>& w_E) const {
    return Vector3<T>(get_symmetric_matrix_view() * w_E);
  }

  /// Sets `this` rotational inertia so all its elements are equal to NaN.
  /// This helps quickly detect uninitialized moments/products of inertia.
  /// Computating with NaN triggers a trackable stack of invalid operations.
  void SetToNaN() {
    I_SP_E_.setConstant(std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  }

  /// Sets `this` rotational inertia so all its moments/products of inertia
  /// are zero, e.g., for convenient initialization before a computation or
  /// for inertia calculations involving a particle (point-mass).
  /// Note: Real 3D massive physical objects have non-zero moments of inertia.
  void SetZero() {
    // Only set the lower-triangular part of this symmetric matrix to zero.
    // The three upper off-diagonal products of inertia should be/remain NaN.
    I_SP_E_.template triangularView<Eigen::Lower>() = Matrix3<T>::Zero();
  }

  /// Returns `true` if any moment/product in `this` rotational inertia is NaN.
  /// Otherwise returns `false`.
  bool IsNaN() const {
    using std::isnan;
    // Only check the lower-triangular part of this symmetric matrix for NaN.
    // The three upper off-diagonal products of inertia should be/remain NaN.
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j <= i; ++j) {
        if (isnan(I_SP_E_(i, j))) return true;
      }
    }
    return false;
  }

  /// This method takes `this` rotational inertia about-point P, expressed-in-
  /// frame E, and computes its principal moments of inertia about-point P, but
  /// expressed-in a frame aligned with the principal axes.
  /// In debug builds, throws std::runtime_error if any moment or products of
  /// inertia in `this` cannot be converted to a double.
  ///
  /// @note This method only works if all moments/products of inertia in `this`
  ///       with a scalar type T can be converted to a double (discarding
  ///       supplemental scalar data such as derivatives of an AutoDiffScalar).
  ///       It fails at runtime if type T cannot be converted to `double`.
  ///
  /// @retval principal_moments The vector of principal moments of inertia
  ///                           `[Ixx Iyy Izz]` sorted in ascending order.
  /// @throws std::runtime_error if eigenvalue solver fails.
  Vector3<double> CalcPrincipalMomentsOfInertia() const {
    // Notes:
    //   1. Eigen's SelfAdjointEigenSolver does not compile for AutoDiffScalar.
    //      Therefore, convert `this` to a local copy of type Matrix3<double>.
    //   2. Eigen's SelfAdjointEigenSolver only uses the lower-triangular part
    //      of this symmetric matrix.
    Matrix3<double> Id;
    Id(0, 0) = ExtractDoubleOrThrow(I_SP_E_(0, 0));
    Id(1, 0) = ExtractDoubleOrThrow(I_SP_E_(1, 0));
    Id(2, 0) = ExtractDoubleOrThrow(I_SP_E_(2, 0));
    Id(1, 1) = ExtractDoubleOrThrow(I_SP_E_(1, 1));
    Id(2, 1) = ExtractDoubleOrThrow(I_SP_E_(2, 1));
    Id(2, 2) = ExtractDoubleOrThrow(I_SP_E_(2, 2));

    // If all products of inertia are zero, no need to calculate eigenvalues.
    // The eigenvalues are the diagonal elements.  Sort them in ascending order.
    const bool is_diagonal = (Id(1, 0) == 0 && Id(2, 0) == 0 && Id(2, 1) == 0);
    if (is_diagonal) {
      Vector3<double> moments(Id(0, 0), Id(1, 1), Id(2, 2));
      std::sort(moments.data(), moments.data()+moments.size());
      return moments;
    }

    Eigen::SelfAdjointEigenSolver<Matrix3<double>> solver(
        Id, Eigen::EigenvaluesOnly);
    if (solver.info() != Eigen::Success) {
      throw std::runtime_error(
          "Error: In RotationalInertia::CalcPrincipalMomentsOfInertia()."
          " Solver failed while computing eigenvalues of the inertia matrix.");
    }
    return solver.eigenvalues();
  }

  /// Performs several necessary checks to verify whether `this` rotational
  /// inertia *could* be physically valid, including:
  /// - No NaN moments or products of inertia.
  /// - Ixx, Iyy, Izz and principal moments are all non-negative (not NaN).
  /// - Ixx, Iyy  Izz and principal moments satisfy the triangle inequality:
  ///   - `Ixx + Iyy >= Izz`
  ///   - `Ixx + Izz >= Iyy`
  ///   - `Iyy + Izz >= Ixx`
  ///
  /// In debug builds, throws std::runtime_error if any moment or products of
  /// inertia in `this` cannot be converted to a double.
  ///
  /// @warning These checks are necessary (but NOT sufficient) conditions for a
  /// rotational inertia to be physically valid. The sufficient condition
  /// requires a rotational inertia to satisfy the above checks *after* `this'
  /// is shifed to the center of mass using the parallel axis theorem.  However,
  /// this class does not know its about-point or its center of mass location.
  /// Sufficient conditions require calling CouldBePhysicallyValid() when the
  /// about-point is the body's center of mass.
  ///
  /// @return `true` for a plausible rotational inertia passing the above
  ///          necessary but insufficient checks and `false` otherwise.
  bool CouldBePhysicallyValid() const {
    if (IsNaN()) return false;

    const double trace  = ExtractDoubleOrThrow(CalcTrace());
    if ( trace < 0 ) return false;

    // Calculates principal moments of inertia.
    const Vector3<double> d = CalcPrincipalMomentsOfInertia();

    // Check the validity of rotational inertia using an epsilon value that is
    // a number related to machine precision multiplied by the largest possible
    // element that can appear in a valid `this` rotational inertia.
    const double precision = 10 * std::numeric_limits<double>::epsilon();
    const double max_possible_inertia_moment_or_product = 0.5 * trace;
    const double epsilon = precision * max_possible_inertia_moment_or_product;

    // Test principal moments of inertia to be mostly non-negative and
    // also satisfy triangle inequality.
    return AreMomentsOfInertiaNearPositiveAndSatisfyTriangleInequality(
        d(0), d(1), d(2), epsilon);
  }

  /// Re-expresses `this` rotational inertia `I_SP_E` so it is `I_SP_A`.
  /// In other words, starts with `this` rotational inertia of a body (or
  /// composite body) S about-point P expressed-in-frame E and re-expresses
  /// to S's rotational inertia about-point P expressed-in-frame A, i.e.,
  /// `I_SP_A = R_AE * I_SP_E * (R_AE)ᵀ`.
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// re-expressed-in-frame A violates CouldBePhysicallyValid().
  /// @param R_AE Rotation matrix from frame A to frame E.
  /// @return A reference to `this` rotational inertia about-point P, but
  ///         with `this` expressed in frame A (instead of frame E).
  /// @see ReExpress().
  RotationalInertia<T>& ReExpressInPlace(const Matrix3<T>& R_AE) {
    // There is an interesting discussion on Eigen's forum here:
    // https://forum.kde.org/viewtopic.php?f=74&t=97282
    // That discussion tell us that really here we don't have a significant
    // performance gain for small matrices when writing on one of the triangular
    // parts only. Then gain is in accuracy, by having RotationalInertia to only
    // deal with one triangular portion of the matrix.

    // Local copy to avoid aliasing that occurs if using triangular view.
    Matrix3<T> I_SP_A;
    I_SP_A.noalias() = R_AE *
        I_SP_E_.template selfadjointView<Eigen::Lower>() * R_AE.transpose();

    // Note: There is no guarantee of a symmetric result in I_SP_A (although it
    // should be symmetric within round-off error). Here we discard the upper-
    // triangular elements (which could slightly differ from the lower ones).
    this->get_mutable_triangular_view() = I_SP_A;

    // If both `this` and `R_AE` were valid upon entry to this method, the
    // returned rotational inertia should be valid.  Otherwise, it may not be.
    DRAKE_ASSERT_VOID(AssertCouldBeValid());
    return *this;
  }

  /// Copies `this` rotational inertia `I_SP_E` and re-expresses it to `I_SP_A`,
  /// i.e., re-expresses body S's rotational inertia from frame E to frame A.
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// re-expressed-in-frame A violates CouldBePhysicallyValid().
  /// @param R_AE Rotation matrix from frame A to frame E.
  /// @retval I_SP_A Rotational inertia of B about-point P expressed-in-frame A.
  /// @see ReExpressInPlace()
  RotationalInertia<T> ReExpress(const Matrix3<T>& R_AE) const {
    return RotationalInertia(*this).ReExpressInPlace(R_AE);
  }

  /// Shift `this` rotational inertia for a body (or composite body) B from
  /// about-point Bcm (B's center of mass) to about-point Q.  In other words,
  /// shifts `I_BBcm_E` to `I_BQ_E` (both have expressed-in-frame E).
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// shifted to about-point Q violates CouldBePhysicallyValid().
  /// @param mass The mass of body (or composite body) B.
  /// @param p_BcmQ_E Position vector from Bcm to Q, expressed-in-frame E.
  /// @return A reference to `this` rotational inertia expressed-in-frame E,
  ///         but with `this` shifted from about-point Bcm to about-point Q.
  ///         i.e., returns I_BQ_E,  B's rotational inertia about-point Bcm
  ///         expressed-in-frame E.
  /// @remark Negating the position vector p_BcmQ_E has no affect on the result.
  /// @see ShiftFromCenterOfMassInPlace(), ShiftFromCenterOfMass(),
  ///      ShiftToCenterOfMassInPlace(),   ShiftToCenterOfMass(),
  ///      ShiftToThenAwayFromCenterOfMassInPlace(),
  ///      ShiftToThenAwayFromCenterOfMass().
  RotationalInertia<T>& ShiftFromCenterOfMassInPlace(const T& mass,
                                                   const Vector3<T>& p_BcmQ_E) {
    *this += RotationalInertia(mass, p_BcmQ_E);
    return *this;
  }

  /// Copies `this` rotational inertia for a body B and then shifts from
  /// about-point Bcm (B's center of mass) to about-point Q.  In other words,
  /// shifts `I_BBcm_E` to `I_BQ_E` (both have expressed-in-frame E).
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// shifted to about-point Q violates CouldBePhysicallyValid().
  /// @param mass The mass of body (or composite body) B.
  /// @param p_BcmQ_E Position vector from Bcm to Q, expressed-in-frame E.
  /// @retval I_BQ_E B's rotational inertia about-point Q expressed-in-frame E.
  /// @remark Negating the position vector p_BcmQ_E has no affect on the result.
  /// @see ShiftFromCenterOfMassInPlace(), ShiftFromCenterOfMass(),
  ///      ShiftToCenterOfMassInPlace(),   ShiftToCenterOfMass(),
  ///      ShiftToThenAwayFromCenterOfMassInPlace(),
  ///      ShiftToThenAwayFromCenterOfMass().
  RotationalInertia<T> ShiftFromCenterOfMass(const T& mass,
                                             const Vector3<T>& p_BcmQ_E) const {
    return RotationalInertia(*this).
           ShiftFromCenterOfMassInPlace(mass, p_BcmQ_E);
  }

  /// Shift `this` rotational inertia for a body (or composite body) B from
  /// about-point Q to about-point Bcm (B's center of mass).  In other words,
  /// shifts `I_BQ_E` to `I_BBcm_E` (both are expressed-in-frame E).
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// shifted to about-point Bcm violates CouldBePhysicallyValid().
  /// @param mass The mass of body (or composite body) B.
  /// @param p_QBcm_E Position vector from Q to Bcm, expressed-in-frame E.
  /// @return A reference to `this` rotational inertia expressed-in-frame E,
  ///         but with `this` shifted from about-point Q to about-point Bcm,
  ///         i.e., returns I_BBcm_E, B's rotational inertia about-point Bcm
  ///         expressed-in-frame E.
  /// @remark Negating the position vector p_QBcm_E has no affect on the result.
  /// @see ShiftFromCenterOfMassInPlace(), ShiftFromCenterOfMass(),
  ///      ShiftToCenterOfMassInPlace(),   ShiftToCenterOfMass(),
  ///      ShiftToThenAwayFromCenterOfMassInPlace(),
  ///      ShiftToThenAwayFromCenterOfMass().
  RotationalInertia<T>& ShiftToCenterOfMassInPlace(const T& mass,
                                                   const Vector3<T>& p_QBcm_E) {
    *this -= RotationalInertia(mass, p_QBcm_E);
    return *this;
  }

  /// Copies `this` rotational inertia for a body B and shifts the copy from
  /// about-point Q to about-point Bcm (B's center of mass).  In other words,
  /// shifts `I_BQ_E` to `I_BBcm_E` (both are expressed-in-frame E).
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// shifted to about-point Bcm violates CouldBePhysicallyValid().
  /// @param mass The mass of body (or composite body) B.
  /// @param p_QBcm_E Position vector from Q to Bcm, expressed-in-frame E.
  /// @retval I_BBcm_E B's rotational inertia about-point Bcm
  ///         expressed-in-frame E.
  /// @remark Negating the position vector p_QBcm_E has no affect on the result.
  /// @see ShiftFromCenterOfMassInPlace(), ShiftFromCenterOfMass(),
  ///      ShiftToCenterOfMassInPlace(),   ShiftToCenterOfMass(),
  ///      ShiftToThenAwayFromCenterOfMassInPlace(),
  ///      ShiftToThenAwayFromCenterOfMass().
  RotationalInertia<T> ShiftToCenterOfMass(const T& mass,
                                           const Vector3<T>& p_QBcm_E) const {
    return RotationalInertia(*this).ShiftToCenterOfMassInPlace(mass, p_QBcm_E);
  }

  /// Shift `this` rotational inertia for a body (or composite body) B from
  /// about-point P to about-point Q via Bcm (B's center of mass).  In other
  /// words shifts `I_BP_E` to `I_BQ_E` (both are expressed-in-frame E).
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// shifted to about-point Q violates CouldBePhysicallyValid().
  /// @param mass The mass of body (or composite body) B.
  /// @param p_PBcm_E Position vector from P to Bcm, expressed-in-frame E.
  /// @param p_QBcm_E Position vector from Q to Bcm, expressed-in-frame E.
  /// @return A reference to `this` rotational inertia expressed-in-frame E,
  ///         but with `this` shifted from about-point P to about-point Q,
  ///         i.e., returns I_BQ_E, B's rotational inertia about-point Q
  ///         expressed-in-frame E.
  /// @remark Negating either (or both) position vectors p_PBcm_E and p_QBcm_E
  ///         has no affect on the result.
  /// @see ShiftFromCenterOfMassInPlace(), ShiftFromCenterOfMass(),
  ///      ShiftToCenterOfMassInPlace(),   ShiftToCenterOfMass(),
  ///      ShiftToThenAwayFromCenterOfMassInPlace(),
  ///      ShiftToThenAwayFromCenterOfMass().
  RotationalInertia<T>& ShiftToThenAwayFromCenterOfMassInPlace(const T& mass,
                       const Vector3<T>& p_PBcm_E, const Vector3<T>& p_QBcm_E) {
    *this += mass * ShiftUnitMassBodyToThenAwayFromCenterOfMass(p_PBcm_E,
                                                                p_QBcm_E);
    return *this;
  }

  /// Copies `this` rotational inertia for a body B and shifts the copy from
  /// about-point P to about-point Q via Bcm (B's center of mass).  In other
  /// words shifts `I_BP_E` to `I_BQ_E` (both are expressed-in-frame E).
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// shifted to about-point Q violates CouldBePhysicallyValid().
  /// @param mass The mass of body (or composite body) B.
  /// @param p_PBcm_E Position vector from P to Bcm, expressed-in-frame E.
  /// @param p_QBcm_E Position vector from Q to Bcm, expressed-in-frame E.
  /// @return I_BQ_E, B's rotational inertia about-point Q expressed-in-frame E.
  /// @remark Negating either (or both) position vectors p_PBcm_E and p_QBcm_E
  ///         has no affect on the result.
  /// @see ShiftFromCenterOfMassInPlace(), ShiftFromCenterOfMass(),
  ///      ShiftToCenterOfMassInPlace(),   ShiftToCenterOfMass(),
  ///      ShiftToThenAwayFromCenterOfMassInPlace(),
  ///      ShiftToThenAwayFromCenterOfMass().
  RotationalInertia<T> ShiftToThenAwayFromCenterOfMass(const T& mass,
                 const Vector3<T>& p_PBcm_E, const Vector3<T>& p_QBcm_E) const {
    return RotationalInertia(*this).ShiftToThenAwayFromCenterOfMassInPlace(
                                    mass, p_PBcm_E, p_QBcm_E);
  }

  /// Calculates the rotational inertia that must be added to account for
  /// shifting the rotational inertia for a unit-mass body (or composite body) B
  /// from about-point P to about-point Q via Bcm (B's center of mass).  In
  /// other words, shifts `I_BP_E` to `I_BQ_E` (both are expressed-in-frame E).
  /// In debug builds, throws std::logic_error if rotational inertia that is
  /// shifted to about-point Bcm or Q violate CouldBePhysicallyValid().
  /// @param p_PBcm_E Position vector from P to Bcm, expressed-in-frame E.
  /// @param p_QBcm_E Position vector from Q to Bcm, expressed-in-frame E.
  /// @return A rotational inertia expressed-in-frame E which when added to
  ///         the rotational inertia I_BP_E produces I_BQ_E.  In other words,
  ///         returns I_BQ_E - I_BP_E, expressed-in-frame E.
  /// @remark Negating either (or both) position vectors p_PBcm_E and p_QBcm_E
  ///         has no affect on the result.
  /// @see ShiftFromCenterOfMassInPlace(), ShiftFromCenterOfMass(),
  ///      ShiftToCenterOfMassInPlace(),   ShiftToCenterOfMass(),
  ///      ShiftToThenAwayFromCenterOfMassInPlace(),
  ///      ShiftToThenAwayFromCenterOfMass().
  RotationalInertia<T> ShiftUnitMassBodyToThenAwayFromCenterOfMass(
                       const Vector3<T>& p_PBcm_E, const Vector3<T>& p_QBcm_E) {
    // Concept: Shifts towards the center of mass then shifts award from it.
    // Math: Shifts away from the center of mass then shifts towards it.
    RotationalInertia shift_away(p_QBcm_E, p_QBcm_E);
    RotationalInertia shift_towards(p_PBcm_E, p_PBcm_E);
    return shift_away.MinusEqualsUnchecked(shift_towards);
  }

  /// Multiplies a nonnegative scalar by `this` rotational inertia (>= 0).
  /// In debug builds, throws std::runtime_error if `nonnegative_scalar` < 0.
  /// @return rotational inertia `I_BP_E` multiplied by `nonnegative_scalar`.
  /// @see operator*=().
  friend RotationalInertia<T> operator*(const T& nonnegative_scalar,
                                        const RotationalInertia<T>& I_BP_E) {
    /// Multiplication of a scalar with a rotational matrix is commutative.
    DRAKE_ASSERT(nonnegative_scalar >= 0);
    return RotationalInertia(I_BP_E) *= nonnegative_scalar;
  }

  /// Multiplies `this` rotational inertia by a nonnegative scalar (>= 0).
  /// In debug builds, throws std::runtime_error if `nonnegative_scalar` < 0.
  /// @return `this` rotational inertia multiplied by `nonnegative_scalar`.
  /// @see operator*=().
  friend RotationalInertia<T> operator*(const RotationalInertia<T>& I_BP_E,
                                        const T& nonnegative_scalar) {
    return RotationalInertia(I_BP_E) *= nonnegative_scalar;
  }

 protected:
  // Subtracts a rotational inertia `I_BP_E` from `this` rotational inertia.
  // This method is mathematically useful - but does not necesssarily produce
  // physically valid result.  This method updates updates `this` as `I_BP_E'
  // is subtracted from `this'.
  // @param I_BP_E Rotational inertia of a body (or composite body) B to
  //        be subtracted from `this` rotational inertia.
  // @return A reference to `this` rotational inertia. `this' changes
  //         since rotational inertia `I_BP_E` has been subtracted from it.
  // @see operator-().
  // @warning This operator may produce an invalid rotational inertia.
  //    Use CouldBePhysicallyValid() to perform necessary (but insufficient)
  //    checks on the physical validity of the resulting rotational inertia.
  //    Sufficient conditions require calling CouldBePhysicallyValid() when the
  //    about-point is the body's center of mass.
  // @note This subtract operator is useful for computing rotational inertia
  // of a body with a hole.  First the rotational inertia of a fully solid
  // body S (without the hole) is calculated, then the rotational inertia of
  // the hole (treated as a massive solid body B) is calculated. The rotational
  // inertia of a composite body C (comprised of S and -B) is computed by
  // subtracting B's rotational inertia from S's rotational inertia.
  // @see operator-=().
  RotationalInertia<T>& MinusEqualsUnchecked(
      const RotationalInertia<T>& I_BP_E) {
    this->get_mutable_triangular_view() -= I_BP_E.get_matrix();
    return *this;
  }

 private:
  // Creates a rotational inertia from a more generic Eigen 3x3 matrix by
  // ignoring the generic matrix's three upper-off diagonal matrix elements.
  // This constructor intentionally does not test CouldBePhysicallyValid().
  // The three upper off-diagonal matrix elements remain equal to NaN.
  explicit RotationalInertia(const Matrix3<T>& I) {
      set_moments_and_products_no_validity_check(I(0, 0),  I(1, 1),  I(2, 2),
                                                 I(1, 0),  I(2, 0),  I(2, 1));
  }

  // Sets this rotational inertia's moments and products of inertia. This method
  // intentionally avoids testing CouldBePhysicallyValid().  Some methods need
  // to be able to form non-physical rotational inertias (which are to be
  // subtracted or added to other rotational inertias to form valid ones).
  void set_moments_and_products_no_validity_check(
      const T& Ixx, const T& Iyy, const T& Izz,
      const T& Ixy, const T& Ixz, const T& Iyz) {
    // Note: The three upper off-diagonal matrix elements remain equal to NaN.
    I_SP_E_(0, 0) = Ixx;  I_SP_E_(1, 1) = Iyy;  I_SP_E_(2, 2) = Izz;
    I_SP_E_(1, 0) = Ixy;  I_SP_E_(2, 0) = Ixz;  I_SP_E_(2, 1) = Iyz;
  }

  // Constructs a rotational inertia for a particle Q whose position vector
  // from about-point P is p_PQ_E = xx̂ + yŷ + zẑ = [x, y, z]_E, where E is the
  // expressed-in-frame.  Particle Q's mass (or unit mass) is included in the
  // second argument.
  // In debug builds, throws std::logic_error if rotational inertia that is
  // constructed from these arguments violate CouldBePhysicallyValid().
  // @param p_PQ_E Position from about-point P to Q, expressed-in-frame E.
  // @param mass_p_PQ_E The mass of particle Q multiplied by `p_PQ_E`.
  //                    If unit mass, this argument is simply p_PQ_E.
  // @return I_QP_E, Q's rotational inertia about-point Q expressed-in-frame E.
  RotationalInertia(const Vector3<T>& mass_p_PQ_E, const Vector3<T>& p_PQ_E) {
    const T& mx = mass_p_PQ_E(0);
    const T& my = mass_p_PQ_E(1);
    const T& mz = mass_p_PQ_E(2);
    const T& x = p_PQ_E(0);
    const T& y = p_PQ_E(1);
    const T& z = p_PQ_E(2);
    const T mxx = mx*x;
    const T myy = my*y;
    const T mzz = mz*z;
    set_moments_and_products_no_validity_check(myy + mzz, mxx + mzz, mxx + myy,
                                               -mx * y,   -mx * z,   -my * z);
    DRAKE_ASSERT_VOID(AssertCouldBeValid());
  }

  // Utility method used to swap matrix indexes (i, j) depending on the
  // TriangularViewInUse portion of this inertia. The swap is performed so that
  // we only use the triangular portion corresponding to TriangularViewInUse.
  static void check_and_swap(int* i, int* j) { if (*i < *j) std::swap(*i, *j); }

  // Returns a constant reference to the underlying Eigen matrix. Notice that
  // since RotationalInertia only uses the lower-triangular portion of its
  // matrix, the three upper off-diagonal matrix elements will be NaN.
  // Most users won't call this method.
  const Matrix3<T>& get_matrix() const { return I_SP_E_; }

  // Returns a const Eigen view expression to the symmetric part of the matrix
  // in use by this RotationalInertia.
  const Eigen::SelfAdjointView<const Matrix3<T>, Eigen::Lower>
  get_symmetric_matrix_view() const {
    return I_SP_E_.template selfadjointView<Eigen::Lower>();
  }

  // Returns a mutable Eigen view expression to the symmetric part of the
  // matrix in use by RotationalInertia.
  // Note: operator=() is not defined for Eigen::SelfAdjointView and therefore
  // we need to return a TriangularView here.
  Eigen::TriangularView<Matrix3<T>, Eigen::Lower>
  get_mutable_triangular_view() {
    return I_SP_E_.template triangularView<Eigen::Lower>();
  }

  // Compares `this' rotational inertia's maximum moment of inertia to `other'
  // rotational inertia's maximum moment of inertia to within `epsilon`.
  // @param   epsilon should be a real non-negative number, with units of
  //          inertia (e.g., kg*m^2).  It is usually small relative to the
  //          maximum moment of inertia for `this` or `other`.
  // @return `true` if the absolute value of each moment/product of inertia
  //          in `this' is within `epsilon` of the corresponding moment/
  //          product absolute value in `other`.  Otherwise returns `false`.
  // @note CalcTrace() / 2 is a rotational inertia's maximum possible element,
  // e.g., consider: epsilon = 1E-9 * CalcTrace()  (where 1E-9 is a heuristic).
  bool IsApproxMomentsAndProducts(const RotationalInertia& other,
                                  const double epsilon) const {
    const Vector3<T> moment_difference = get_moments() - other.get_moments();
    const Vector3<T> product_difference = get_products() - other.get_products();
    const T moment_max = moment_difference.template lpNorm<Eigen::Infinity>();
    const T product_max = product_difference.template lpNorm<Eigen::Infinity>();
    return moment_max <= epsilon && product_max <= epsilon;
  }

  // Tests whether each moment of inertia is non-negative (to within `epsilon`)
  // and tests whether moments of inertia satisfy triangle-inequality.  The
  // triangle-inequality test requires `epsilon` when the sum of two moments are
  // nearly equal to the third one. Example: Ixx = Iyy = 50, Izz = 100.00000001,
  // or Ixx = -0.0001 (negative),  Ixx = 49.9999,  Iyy = 50.
  // A positive (non-zero) epsilon accounts for round-off errors, e.g., from
  // CalcPrincipalMomentsOfInertia() or re-expressing inertia in another frame.
  // @param Ixx, Iyy, Izz moments of inertia for a generic rotational inertia,
  //        (i.e., not necessarily principal moments of inertia).
  // @param epsilon Real positive number that is much smaller than the largest
  //        possible element in a valid rotational inertia.  Heuristically,
  //       `epsilon' should probably be a small multiplier of CalcTrace() / 2.
  // @note Denoting Imin and Imax as the smallest and largest possible moments
  //       of inertia in a valid rotational inertia, denoting Imed as the
  //       intermediate moment of inertia, and denoting tr as the trace of the
  //       rotational inertia (e.g., Ixx + Iyy + Izz), one can prove:
  //       0 <= Imin <= tr/3,   tr/3 <= Imed <= tr/2,   tr/3 <= Imax <= tr/2.
  //       If Imin == 0, then Imed == Imax == tr / 2.
  static bool AreMomentsOfInertiaNearPositiveAndSatisfyTriangleInequality(
      const T& Ixx, const T& Iyy, const T&Izz, const double epsilon) {
    const bool are_moments_near_positive = AreMomentsOfInertiaNearPositive(
        Ixx, Iyy, Izz, epsilon);
    const bool is_triangle_inequality_satisified = Ixx + Iyy + epsilon >= Izz &&
                                                   Ixx + Iyy + epsilon >= Iyy &&
                                                   Iyy + Izz + epsilon >= Ixx;
    return are_moments_near_positive && is_triangle_inequality_satisified;
  }

  // Tests whether each moment of inertia is non-negative (to within epsilon).
  // This test allows for small (equal to -epsilon) negative moments of inertia
  // due to round-off errors, e.g., from CalcPrincipalMomentsOfInertia() and/or
  // re-expressing a rotational inertia in another frame.
  // @param Ixx, Iyy, Izz moments of inertia for a generic rotational inertia,
  //        (i.e., not necessarily principal moments of inertia).
  // @param epsilon Real positive number that is significantly smaller than the
  //        largest possible element in a valid rotational inertia.
  //        Heuristically, `epsilon' is a small multiplier of CalcTrace() / 2.
  static bool AreMomentsOfInertiaNearPositive(
      const T& Ixx, const T& Iyy, const T& Izz, const double epsilon) {
    return Ixx + epsilon >= 0  &&  Iyy + epsilon >= 0  &&  Izz + epsilon >= 0;
  }

  // Throws an exception if a rotational inertia is physically valid.
  void AssertCouldBeValid() {
    if ( !CouldBePhysicallyValid() )
      throw std::logic_error("Warning: Rotational inertia did not pass test: "
                                 "CouldBePhysicallyValid()");
  }

  // Throws an exception if a rotational inertia is multiplied by a negative
  // number - which implies that the resulting rotational inertia is invalid.
  void AssertMultiplyByNonnegativeScalar(const T& nonnegative_scalar) {
    if ( nonnegative_scalar < 0 )
      throw std::logic_error("Warning: Rotational inertia is multiplied by a "
                                 "negative number");
  }

  // The 3x3 inertia matrix is symmetric and its diagonal elements (moments of
  // inertia) and off-diagonal elements (products of inertia) are associated
  // with a body (or composite body) S, an about-point P, and an expressed-in-
  // frame E.  A rotational inertia is ill-defined unless there is a body S,
  // about-point P, and expressed-in-frame E. The user of this class is
  // responsible for tracking the body S, about-point P and expressed-in-frame E
  // (none of these are stored in this class).
  // The rotational inertia class only has data for the inertia matrix, whose
  // elements are initially set to NaN to aid in ensuring only the lower-
  // triangular part of the matrix is used (upper-triangular part is not used).
  Matrix3<T> I_SP_E_{Matrix3<T>::Constant(std::numeric_limits<
      typename Eigen::NumTraits<T>::Literal>::quiet_NaN())};
};

/// Insertion operator to write %RotationalInertia's into a `std::ostream`.
/// Especially useful for debugging.
/// @relates RotationalInertia
template <typename T> inline
std::ostream& operator<<(std::ostream& o,
                         const RotationalInertia<T>& I) {
  int width = 0;
  // Computes largest width so that we can align columns for a prettier format.
  // Idea taken from: Eigen::internal::print_matrix() in Eigen/src/Core/IO.h
  for (int j = 0; j < I.cols(); ++j) {
    for (int i = 0; i < I.rows(); ++i) {
      std::stringstream sstr;
      sstr.copyfmt(o);
      sstr << I(i, j);
      width = std::max<int>(width, static_cast<int>(sstr.str().length()));
    }
  }

  // Outputs to stream.
  for (int i = 0; i < I.rows(); ++i) {
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
  return o;
}

}  // namespace multibody
}  // namespace drake
