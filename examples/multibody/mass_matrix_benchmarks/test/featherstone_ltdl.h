#pragma once

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace test {

// LTDL factorization using the algorithm in [Featherstone, 2005].
//
// @param[in] lambda
//   Expanded parent array λ(i) in [Featherstone, 2005]. λ(i) corresponds to the
//   parent node of node i in the connectivity tree.
// @param[in/out] Ainout
//   On input, matrix to be factorized: For each row i the non-zero elements
//   below the main diagonal appear only in columns λ(i), λ(λ(i)), and so on.
//   On output, the strictly lower diagonal stores L (assumed to have ones in
//   the diagonal) and the diagonal contains D. The strictly upper diagonal is
//   not touched.
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
