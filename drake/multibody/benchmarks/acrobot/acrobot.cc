#include "drake/multibody/benchmarks/acrobot/acrobot.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace multibody {
namespace benchmarks {

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

template <typename T>
Acrobot<T>::Acrobot(const Vector3<T>& normal, const Vector3<T>& up) {
  // Asserts that neither normal nor up are zero vectors.
  DRAKE_ASSERT(!normal.isZero());
  DRAKE_ASSERT(!up.isZero());
  // Asserts that normal and up are not in the same line.
  DRAKE_ASSERT(
      std::abs(std::abs(ExtractDoubleOrThrow(normal.dot(up))) - 1.0) >
          Eigen::NumTraits<double>::epsilon());

  Vector3<T> z_W = normal.normalized();
  Vector3<T> y_W = (up - up.dot(z_W) * z_W).normalized();
  Vector3<T> x_W = y_W.cross(z_W);
  // Rotation transformation from the model frame D to the world frame W.
  Matrix3<T> R_WD;
  R_WD.col(0) = x_W;
  R_WD.col(1) = y_W;
  R_WD.col(2) = z_W;
  X_WD_.linear() = R_WD;

  PRINT_VAR(x_W.transpose());
  PRINT_VAR(y_W.transpose());
  PRINT_VAR(z_W.transpose());
  PRINT_VAR(X_WD_);
}

template <typename T>
Matrix2<T> Acrobot<T>::CalcMassMatrix(const T& theta1, const T& theta2)
const {
  const T I1 = Ic1 + m1 * lc1 * lc1;
  const T I2 = Ic2 + m2 * lc2 * lc2;
  const T m2l1lc2 = m2 * l1 * lc2;  // occurs often!

  using std::sin;
  using std::cos;
  const T c2 = cos(theta2);

  const T h12 = I2 + m2l1lc2 * c2;
  Matrix2<T> H;
  H << I1 + I2 + m2 * l1 * l1 + 2 * m2l1lc2 * c2, h12, h12, I2;
  return H;
}

#if 0
template <typename T>
Isometry3<T> Acrobot<T>::CalcLinkPoseInWorldFrame(
    const T& theta1, const T& theta2, int link_id) const {

}
#endif

template <typename T>
Isometry3<T> Acrobot<T>::CalcLink1PoseInWorldFrame(
    const T& theta1, const T& theta2) const {
  using std::sin;
  using std::cos;

  // Perform computation in model frame D.

  // Center of mass position of link 1 in the model frame D.
  Vector3<T> xcm1_D = lc1 * Vector3<T>(sin(theta1), -cos(theta1), 0.0);

  // Pose of link 1 frame measured and expressed in D.
  Isometry3<T> X_DL1;
  X_DL1.linear() = Matrix3<T>(AngleAxis<T>(theta1, Vector3<T>::UnitZ()));
  X_DL1.translation() = xcm1_D;

  // Transformation to world frame W.
  return X_WD_ * X_DL1;
}

template class Acrobot<double>;
template class Acrobot<AutoDiffXd>;

}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
