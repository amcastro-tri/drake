#include "drake/math/rotation_matrix_with_derivatives.h"

#include "drake/math/autodiff.h"

namespace drake {
namespace math {
namespace internal {

using Eigen::Matrix3d;

bool IsNearlyEqualTo(const RotationMatrixWithDerivatives& m1,
                     const RotationMatrixWithDerivatives& m2,
                     double tolerance) {
  if (m1.derivatives().size() != m2.derivatives().size()) {
    return false;
  }
  if ((m1.value() - m2.value()).template lpNorm<Eigen::Infinity>() > tolerance) {
    return false;
  }
  for (int i = 0; i < m1.num_derivatives(); ++i) {    
    if ((m1.derivatives()[i] - m2.derivatives()[i])
            .template lpNorm<Eigen::Infinity>() > tolerance) {
      return false;
    }
  }
  return true;
}

bool operator==(const RotationMatrixWithDerivatives& m1,
                const RotationMatrixWithDerivatives& m2) {
  return IsNearlyEqualTo(m1, m2, 0.0);
}

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

Vector3<AutoDiffXd> RotationMatrixWithDerivatives::operator*(
    const Vector3<AutoDiffXd>& v_B) const {
  Vector3<AutoDiffXd> ret(value() * v_B);
  Eigen::Vector3d v_B_value{v_B[0].value(), v_B[1].value(), v_B[2].value()};
  for (int row = 0; row < 3; ++row) {
    if (ret[row].derivatives().size() == 0) {
      ret[row].derivatives() = Eigen::RowVectorXd::Zero(num_derivatives());
    }
  }
  for (int i = 0; i < num_derivatives(); ++i) {
    for (int row = 0; row < 3; ++row) {
      ret[row].derivatives()[i] += derivatives()[i].row(row).dot(v_B_value);
    }
  }
  return ret;
}

RotationMatrixWithDerivatives RotationMatrixWithDerivatives::MakeZRotation(
    const AutoDiffXd& theta) {
  using std::cos;
  using std::sin;

  const double c = cos(theta.value()), s = sin(theta.value());
  Eigen::Matrix3d R_AB = Eigen::Matrix3d::Zero();
  R_AB(0, 0) = c, R_AB(0, 1) = -s;
  R_AB(1, 0) = s, R_AB(1, 1) = c;

  const int num_derivs = theta.derivatives().size();
  std::vector<Eigen::Matrix3d> dRdv(num_derivs, Eigen::Matrix3d::Zero());
  if (num_derivs > 0) {
    const Eigen::Vector3d w = Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d wR = R_AB.colwise().cross(w);
    for (int i = 0; i < num_derivs; ++i) {
      if (theta.derivatives()[i] != 0.0) {
        dRdv[i] = wR * theta.derivatives()[i];
      }
    }
  }
  return RotationMatrixWithDerivatives(std::move(R_AB), std::move(dRdv));
}


}  // namespace internal
}  // namespace math
}  // namespace drake
