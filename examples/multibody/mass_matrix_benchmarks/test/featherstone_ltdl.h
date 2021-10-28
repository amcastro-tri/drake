#pragma once

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace test {

void CalcLtdlInPlace(const std::vector<int>& lambda, Eigen::MatrixXd* Ainout) {
  DRAKE_DEMAND(Ainout->rows() == Ainout->cols());
  auto& A = *Ainout;
  const int n = A.rows();
  for (int k = n - 1; k >= 0; --k) {
    int i = lambda[k];
    while (i >= 0) {
      const double a = A(k, i) / A(k, k);
      int j = i;
      while (j >= 0) {
        A(i, j) -= a * A(k, j);
        j = lambda[j];
      }
      A(k, i) = a;
      i = lambda[i];
    }
  }
}

}  // namespace test
}  // namespace examples
}  // namespace drake
