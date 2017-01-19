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

#if 0
/// A column vector of size 6 consisting of a concatenating rotational and
/// translational components of a physical quantity, see
/// @ref multibody_spatial_algebra
template<typename Scalar>
using SpatialVector = Eigen::Matrix<Scalar, 6, 1>;
#endif

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

// Base generic SpatialVector with common functionality to both 2D and 3D.
template <class SpatialVector>
class SpatialVectorBase {
 public:
  typedef typename internal::traits<SpatialVector>::Vec V;
  enum {
    kNumDimensions = VectorSpace<V>::kNumDimensions,
    kSpatialVectorSize = VectorSpace<V>::kSpatialVectorSize,
    kSpatialVectorAngularSize = VectorSpace<V>::kSpatialVectorAngularSize,
    kSpatialVectorLinearSize = VectorSpace<V>::kSpatialVectorLinearSize
  };
  typedef typename VectorSpace<V>::ScalarType T;
  typedef typename VectorSpace<V>::Spin Spin;  // Type of the angular component.
  typedef typename VectorSpace<V>::Vec Vec;  // Type of the linear component.
  typedef Eigen::Matrix<T, kSpatialVectorSize, 1> CoeffsEigenType;

  SpatialVectorBase(const Spin& w, const Vec& v) {
    V_ << w, v;
  }

  int size() const { return kSpatialVectorSize; }
  int angular_size() const { return kSpatialVectorAngularSize; }
  int linear_size() const { return kSpatialVectorLinearSize; }

  const CoeffsEigenType& get_coeffs() const { return V_;}

  T& operator[](int i) { return V_[i];}
  const T& operator[](int i) const { return V_[i];}

  const Spin& angular() const {
    return *reinterpret_cast<const Spin*>(V_.data());
  }

  Spin& angular() {
    return *reinterpret_cast<Spin*>(V_.data());
  }

  const Vec& linear() const {
    return *reinterpret_cast<const Vec*>(V_.data() + kSpatialVectorAngularSize);
    //return *reinterpret_cast<const Vec*>(&V_[kSpatialVectorAngularSize]);
    //return *(new (V_.data()+kSpatialVectorAngularSize) Vec());
  }

  Vec& linear() {
    return *reinterpret_cast<Vec*>(V_.data() + kSpatialVectorAngularSize);
    //return *reinterpret_cast<Vec*>(&V_[kSpatialVectorAngularSize]);
    //return *(new (V_.data()+kSpatialVectorAngularSize) Vec());
  }

 protected:
  CoeffsEigenType V_;
};

template <class Vec> inline
std::ostream& operator<<(std::ostream& o, const SpatialVector<Vec>& V) {
  o << "[" << V[0];
  for(int i = 1; i < V.size(); ++i) o << ", " << V[i];
  o << "]^T";  // The "transpose" symbol.
  return o;
}

// General implementation.
template <class V>
class SpatialVector :
    public SpatialVectorBase<SpatialVector<V>> {
 public:
  typedef SpatialVectorBase<SpatialVector<V>> Base;
  typedef typename VectorSpace<V>::ScalarType T;
  typedef typename VectorSpace<V>::Spin Spin;  // Type of the angular component.
  typedef typename VectorSpace<V>::Vec Vec;  // Type of the linear component.

  SpatialVector(const Spin& w, const Vec& v) : Base(w, v) {}
};

// Specialization for 2D.
template <typename T>
class SpatialVector<Vector2<T>> :
    public SpatialVectorBase<SpatialVector<Vector2<T>>> {
 public:
  typedef Vector2<T> V;
  enum {
    kNumDimensions = VectorSpace<V>::kNumDimensions,
    kSpatialVectorSize = VectorSpace<V>::kSpatialVectorSize,
    kSpatialVectorAngularSize = VectorSpace<V>::kSpatialVectorAngularSize,
    kSpatialVectorLinearSize = VectorSpace<V>::kSpatialVectorLinearSize
  };
  typedef SpatialVectorBase<SpatialVector<Vector2<T>>> Base;
  typedef typename VectorSpace<V>::Spin Spin;
  typedef typename VectorSpace<V>::Vec Vec;
  typedef Eigen::Matrix<T, 2, 1, Eigen::DontAlign> NonAlignedVec;

  SpatialVector(const Spin& w, const Vec& v) : Base(w, v) {}

  // These accessors overwrite the default implementation in SpatialVectorBase.
  // They are made to return non-aligned 2D vectors since it was found that
  // when doing:
  //   std::cout << V_XY.linear().transpose();
  // a segfault occurred.
  const NonAlignedVec& linear() const {
    return *reinterpret_cast<const NonAlignedVec*>(
        V_.data() + kSpatialVectorAngularSize);
  }

  NonAlignedVec& linear() {
    return *reinterpret_cast<NonAlignedVec*>(
        V_.data() + kSpatialVectorAngularSize);
  }

 private:
  using Base::V_;
};

#if 0
/// A class representing the Rigid Body Transformation Matrix as defined in
/// Section 1.4 of A Jain's book.
/// Given a pair of frames `x` and `y`
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class PhiMatrix {
 public:
  /// Constructs a rigid body transformation operator between a pair of frames
  /// `X` and `Y` given a vector @p l from `Xo` to `Yo`. For a vector @p l_F
  /// expressed in a frame `F`, this operator can only operate on spatial
  /// vectors expressed in the same frame `F`.
  /// @param[in] l_F Vector from `Xo` to `Yo` expressed in an implicit
  /// frame `F`.
  PhiMatrix(const Vector3<T> l_F) : l_F_(l_F) {}

  // #define ConstRef(Vector) const Ref<const Vector>& ??

  /// Given the spatial velocity of a frame `B` with respect to a frame `A`,
  /// computed the spatial velocity of a frame `Q` rigidly moving with `B` but
  /// offset by vector `r_BA`.
  /// @param[in] V_AB Spatial velocity of frame `B` with respect to frame `A`,
  /// expressed in frame `A`.
  /// @param[in] r_BQ_A Shift vector from `Bo` to `Qo`, expressed in frame `A`.
  /// @returns V_AQ_A The spatial velocity of frame `Q` with respect to `A` and
  /// expressed in frame `A`.
  static SpatialVector ShiftVelocity(const Ref<const SpatialVector>& V_AB, const Ref<const Vector3<T>>& r_BQ_A) {
    SpatialVector V_AQ_A;
    // Same angular velocity.
    V_AQ_A.template topRows<3>() = V_AB.template topRows<3>();
    // Linear velocity v_AQ = v_AB + w_AB.cross(r_BQ).
    V_AQ_A.template bottomRows<3>().noalias() =
        V_AB.template bottomRows<3>() +
        V_AB.template topRows<3>().colwise().cross(r_BQ_A);
  }
 private:
  Vector3<T> l_F_;
};
#endif

}  // math
}  // namespace multibody
}  // namespace drake
