#include "drake/math/rotation_matrix_with_derivatives.h"

#include "drake/math/autodiff.h"

namespace drake {
namespace math {
namespace internal {

using Eigen::Matrix3d;

RotationMatrixWithDerivatives::RotationMatrixWithDerivatives(
    const Matrix3<AutoDiffXd>& M) {
  int num_derivatives = 0;
  Matrix3d value;
  std::vector<Matrix3d> derivatives;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      value(row, col) = M(row, col).value();
      if (num_derivatives == 0) {
        num_derivatives = M(row, col).derivatives().size();
      } else {
        DRAKE_THROW_UNLESS(num_derivatives ==
                               M(row, col).derivatives().size() ||
                           M(row, col).derivatives().size() == 0);
      }
    }
  }

  derivatives.resize(num_derivatives);
  for (int deriv = 0; deriv < num_derivatives; ++deriv) {
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        double d = M(row, col).derivatives().size() == 0
                       ? 0.0
                       : M(row, col).derivatives()[deriv];
        derivatives[deriv](row, col) = d;
      }
    }
  }
  Reset(std::move(value), std::move(derivatives));
}

Matrix3<AutoDiffXd> RotationMatrixWithDerivatives::ToAutoDiffXd() const {
  Matrix3<AutoDiffXd> M = value().cast<AutoDiffXd>();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      for (int deriv = 0; deriv < num_derivatives(); ++deriv) {
        if (M(row, col).derivatives().size() == 0) {
          M(row, col).derivatives() =
              Eigen::VectorXd::Zero(num_derivatives());
        }
        M(row, col).derivatives()(deriv) = derivatives()[deriv](row, col);
      }
    }
  }
  return M;
}

RotationMatrixWithDerivatives RotationMatrixWithDerivatives::transpose() const {
  Matrix3d v = value().transpose();
  std::vector<Matrix3d> d(num_derivatives());
  for (int i = 0; i < num_derivatives(); ++i) {
    d[i] = derivatives()[i].transpose();
  }
  return RotationMatrixWithDerivatives(std::move(v), std::move(d));
}

bool RotationMatrixWithDerivatives::IsExactlyIdentity() const {
  if (value() != Matrix3d::Identity()) {
    return false;
  }
  // TODO: Move this to ObjectWithDerivatives::HasZeroDerivatives(), so that the
  // implementaiton is independent of dense or sparse storage.
  for (const auto& d : derivatives()) {
    if (d != Matrix3d::Zero()) return false;
  }
  return true;
}

}  // namespace internal
}  // namespace math
}  // namespace drake
