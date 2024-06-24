#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/math/spatial_algebra.h"

namespace drake {

// Computes b += β⋅a×eᵢ, where β is a scalar and eᵢ is the basis vector along
// the i-th axis.
template <typename T, int axis>
struct CrossProductWithBasisVector {
  static void CalcAndAdd(const Vector3<T>& a, const T& beta, Vector3<T>* b_ptr);
};

template <typename T>
struct CrossProductWithBasisVector<T, 0> {
  static void CalcAndAdd(const Vector3<T>& a, const T& beta,
                         Vector3<T>* b_ptr) {
    Vector3<T>& b = *b_ptr;
    b(1) += beta * a(2);
    b(2) -= beta * a(1);
  }
};

template <typename T>
struct CrossProductWithBasisVector<T, 1> {
  static void CalcAndAdd(const Vector3<T>& a, const T& beta,
                         Vector3<T>* b_ptr) {
    Vector3<T>& b = *b_ptr;
    b(0) -= beta * a(2);
    b(2) += beta * a(0);
  }
};

template <typename T>
struct CrossProductWithBasisVector<T, 2> {
  static void CalcAndAdd(const Vector3<T>& a, const T& beta,
                         Vector3<T>* b_ptr) {
    Vector3<T>& b = *b_ptr;
    b(0) += beta * a(1);
    b(1) -= beta * a(0);
  }
};

namespace multibody {
namespace internal {

// template <typename T, int axis>
// using RevoluteMobodSpatialMotion = SpatialVelocity<T>;

template <typename T, int axis>
class RevoluteMobodRigidTransform {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RevoluteMobodRigidTransform);

  // Identity transform.
  RevoluteMobodRigidTransform() = default;

  // Rotation of `theta` about `axis`.
  RevoluteMobodRigidTransform(const T& theta) {
    if constexpr (axis == 0) {
      transform_ = math::RigidTransform<T>(
          math::RotationMatrix<T>::MakeXRotation(theta));
    } else if constexpr (axis == 1) {
      transform_ = math::RigidTransform<T>(
          math::RotationMatrix<T>::MakeYRotation(theta));
    } else {
      transform_ = math::RigidTransform<T>(
          math::RotationMatrix<T>::MakeZRotation(theta));
    }
  }

 private:
  template <typename U, int an_axis>
  friend math::RigidTransform<U> operator*(
      const math::RigidTransform<U>& X_AB,
      const RevoluteMobodRigidTransform<U, an_axis>& X_BC);

  // TODO: optimize to take advantage of being a rotation around axis.
  math::RigidTransform<T> transform_;
};

// Compose rigid transforms as X_AC = X_AB * X_BC.
template <typename T, int axis>
math::RigidTransform<T> operator*(
    const math::RigidTransform<T>& X_AB,
    const RevoluteMobodRigidTransform<T, axis>& X_BC) {
  // TODO: optimize algebra for simple rotation.
  return X_AB * X_BC.transform_;
}

template <typename T, int axis>
struct RevoluteMobodPositionKinematicsData {
  // sine and cosine of the scalar configuration q (angle).
  // Default initialized to values corresponding to zero angle.
  T sin_q{0.0}, cos_q{1.0};
  RevoluteMobodRigidTransform<T, axis> X_FM;
};

template <typename T, int axis>
class RevoluteMobodSpatialMotion {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RevoluteMobodSpatialMotion);

  // Zero spatial velocity.
  RevoluteMobodSpatialMotion() = default;

  // SpatialVelocity with rotational rate `theta_dot` about `axis`.
  RevoluteMobodSpatialMotion(const T& theta_dot) : theta_dot_(theta_dot) {}

  const T& theta_dot() const { return theta_dot_; }

  // Computes the acceleration bias needed to compose parent frame acceleration
  // A_WP_P with mobilized frame acceleration A_FM_M. This is:
  //  Ab_M = |    w_WM_M x w_FM_M |  = |                w_WM_M x w_FM_M |
  //         |2 * w_WP_M x v_FM_M |    | 2 * (w_WM_M - w_FM_M) x v_FM_M |
  // Notice this computation is greatly simplified when:
  //  1. Either v_FM_M or w_FM_M is zero.
  //  2. When v_FM_M and/or w_FM_M only have a single non-zero component.
  // As an example, for revolute joints, v_FM_M is zero and only one
  // component in v_FM_M is non-zero. Similarly for prismatic joints.
  void AccumulateAccelerationBias(const Vector3<T>& w_WM_M,
                                  SpatialAcceleration<T>* Ab_M) const {
    // In this case v_FM = 0, thus 2 * w_WP_M x v_FM_M = 0.
    const Vector3<T>& alpha_bias = Ab_M->rotational();
    // alpha_bias += w_WM_M x w_FM_M ( = theta_dot ⋅ w_WM_M×eᵢ)
    CrossProductWithBasisVector<T, axis>::CalcAndAdd(w_WM_M, theta_dot_,
                                                     &alpha_bias);
  }

 private:
  // and are expressed in the same frame, W.
  template <template <typename> class SpatialMotion, typename U, int an_axis>
  friend SpatialMotion<U> operator+(
      const SpatialMotion<U>& V_WPb,
      const RevoluteMobodSpatialMotion<U, an_axis>& V_PB_W);

  T theta_dot_{0.0};
};

// Compose spatial velocities as V_WB = V_WPb + V_PB_W.
// @pre both operands contribute to the spatial velocity of the same point, B,
// and are expressed in the same frame, W.
template <template <typename> class SpatialMotion, typename T, int axis>
SpatialMotion<T> operator+(const SpatialMotion<T>& V_WPb,
                           const RevoluteMobodSpatialMotion<T, axis>& V_PB_W) {
  static_assert(std::is_same_v<SpatialMotion<T>, SpatialVelocity<T>> ||
                std::is_same_v<SpatialMotion<T>, SpatialAcceleration<T>>);
  SpatialMotion<T> V_WB(V_WPb);
  V_WB[axis] += V_PB_W.theta_dot_;
  return V_WB;
}

template <typename T, int axis>
struct RevoluteMobodVelocityKinematicsData {
  RevoluteMobodSpatialMotion<T, axis> V_FM;
};

template <typename T, int axis>
struct RevoluteMobodAccelerationKinematicsData {
  RevoluteMobodSpatialMotion<T, axis> A_FM_M;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
